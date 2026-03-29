#!/usr/bin/env python3
"""
AVB Audio Analyzer - Extract and analyze PCM audio from AVTP packet captures.

Reads pcap/pcapng files containing AVTP audio streams (AAF PCM or IEC 61883-6
AM824 format), extracts raw PCM samples, optionally writes a WAV file, and
performs signal analysis to detect glitches, silence gaps, periodic artifacts,
SNR, and THD.

Protocol references:
  - IEEE 1722-2016 (AVTP) Section 7 (AAF) and Section 9 (IEC 61883)

Usage:
  python3 avb_audio_analyzer.py capture.pcap
  python3 avb_audio_analyzer.py capture.pcap --wav output.wav
  python3 avb_audio_analyzer.py capture.pcap --src-mac 00:11:22:33:44:55
  python3 avb_audio_analyzer.py capture.pcap --channel 1
"""

import argparse
import math
import os
import struct
import subprocess
import sys
import tempfile
import wave

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ETH_P_AVTP = 0x22F0
ETH_P_VLAN = 0x8100

AVTP_SUBTYPE_61883 = 0x00
AVTP_SUBTYPE_AAF = 0x02

# IEC 61883-6 CIP header length
CIP_HEADER_LEN = 8

# AM824 label for multi-bit linear audio
AM824_LABEL_MBLA = 0x40

# Pcap magic numbers
PCAP_MAGIC_LE = 0xA1B2C3D4
PCAP_MAGIC_BE = 0xD4C3B2A1
PCAP_MAGIC_NS_LE = 0xA1B23C4D
PCAP_MAGIC_NS_BE = 0x4D3CB2A1
PCAPNG_MAGIC = 0x0A0D0D0A  # Section Header Block type

# Ethernet link type
LINKTYPE_ETHERNET = 1


# ---------------------------------------------------------------------------
# Pcap reader (handles classic pcap; pcapng converted via editcap)
# ---------------------------------------------------------------------------

def _convert_pcapng_to_pcap(path):
    """Convert pcapng to pcap using editcap, return temp file path."""
    tmp = tempfile.NamedTemporaryFile(suffix=".pcap", delete=False)
    tmp.close()
    try:
        subprocess.check_call(["editcap", "-F", "pcap", path, tmp.name],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        print("ERROR: editcap not found. Install wireshark-cli or tshark to convert pcapng files.",
              file=sys.stderr)
        os.unlink(tmp.name)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"ERROR: editcap failed: {e}", file=sys.stderr)
        os.unlink(tmp.name)
        sys.exit(1)
    return tmp.name


def _is_pcapng(path):
    """Check if a file is pcapng format."""
    with open(path, "rb") as f:
        magic = f.read(4)
        if len(magic) < 4:
            return False
        val = struct.unpack("<I", magic)[0]
        return val == PCAPNG_MAGIC


def read_pcap_packets(path):
    """
    Yield (timestamp_sec, raw_packet_bytes) from a pcap file.
    Automatically converts pcapng via editcap.
    """
    tmp_path = None
    if _is_pcapng(path):
        tmp_path = _convert_pcapng_to_pcap(path)
        path = tmp_path

    try:
        with open(path, "rb") as f:
            # Global header: 24 bytes
            ghdr = f.read(24)
            if len(ghdr) < 24:
                raise ValueError("Truncated pcap global header")

            magic = struct.unpack("<I", ghdr[:4])[0]
            if magic == PCAP_MAGIC_LE:
                endian = "<"
                ts_ns = False
            elif magic == PCAP_MAGIC_BE:
                endian = ">"
                ts_ns = False
            elif magic == PCAP_MAGIC_NS_LE:
                endian = "<"
                ts_ns = True
            elif magic == PCAP_MAGIC_NS_BE:
                endian = ">"
                ts_ns = True
            else:
                raise ValueError(f"Not a pcap file (magic=0x{magic:08X})")

            _ver_maj, _ver_min, _tz, _sigfigs, snaplen, linktype = struct.unpack(
                endian + "HHiIII", ghdr[4:])

            if linktype != LINKTYPE_ETHERNET:
                raise ValueError(f"Unsupported link type {linktype}, expected Ethernet ({LINKTYPE_ETHERNET})")

            # Read packets
            while True:
                phdr = f.read(16)
                if len(phdr) < 16:
                    break
                ts_sec, ts_frac, incl_len, orig_len = struct.unpack(endian + "IIII", phdr)
                data = f.read(incl_len)
                if len(data) < incl_len:
                    break
                if ts_ns:
                    timestamp = ts_sec + ts_frac / 1e9
                else:
                    timestamp = ts_sec + ts_frac / 1e6
                yield (timestamp, data)
    finally:
        if tmp_path:
            os.unlink(tmp_path)


# ---------------------------------------------------------------------------
# AVTP packet parsing
# ---------------------------------------------------------------------------

def parse_mac(data, offset):
    """Return MAC address string from 6 bytes."""
    return ":".join(f"{b:02x}" for b in data[offset:offset+6])


def extract_avtp_audio(pcap_path, src_mac_filter=None, channel=0,
                       sample_rate=48000, bit_depth=24):
    """
    Extract PCM audio samples from AVTP packets in a pcap file.

    Returns:
        samples: list of int (signed 24-bit values)
        packet_info: list of dicts with per-packet metadata
        stream_info: dict with detected stream parameters
    """
    samples = []
    packet_info = []
    first_ts = None
    seq_prev = None
    seq_gaps = 0
    total_packets = 0
    format_name = None
    detected_channels = None
    detected_rate = None

    for ts, raw in read_pcap_packets(pcap_path):
        if len(raw) < 14:
            continue

        # Parse Ethernet header
        dst_mac = raw[0:6]
        src_mac = raw[6:12]
        ethertype = struct.unpack("!H", raw[12:14])[0]
        offset = 14

        # Handle VLAN tag
        if ethertype == ETH_P_VLAN:
            if len(raw) < 18:
                continue
            # Skip TCI (2 bytes), read inner ethertype
            ethertype = struct.unpack("!H", raw[16:18])[0]
            offset = 18

        if ethertype != ETH_P_AVTP:
            continue

        # Filter by source MAC
        if src_mac_filter:
            mac_str = parse_mac(raw, 6)
            if mac_str.lower() != src_mac_filter.lower():
                continue

        if len(raw) < offset + 1:
            continue

        subtype = raw[offset]

        if subtype == AVTP_SUBTYPE_AAF:
            pkt_samples = _parse_aaf_packet(raw, offset, channel, bit_depth)
            if pkt_samples is None:
                continue
            samps, nch, sr = pkt_samples
            if format_name is None:
                format_name = "AAF"
                detected_channels = nch
                detected_rate = sr
        elif subtype == AVTP_SUBTYPE_61883:
            pkt_samples = _parse_61883_packet(raw, offset, channel)
            if pkt_samples is None:
                continue
            samps, nch = pkt_samples
            if format_name is None:
                format_name = "IEC 61883-6 AM824"
                detected_channels = nch
                detected_rate = None  # not encoded in packet header for 61883
        else:
            continue

        total_packets += 1

        # Sequence number tracking
        seq_num = raw[offset + 2]
        if seq_prev is not None:
            expected = (seq_prev + 1) & 0xFF
            if seq_num != expected:
                seq_gaps += 1
        seq_prev = seq_num

        if first_ts is None:
            first_ts = ts

        rel_ts = ts - first_ts

        packet_info.append({
            "timestamp": rel_ts,
            "seq_num": seq_num,
            "num_samples": len(samps),
            "sample_offset": len(samples),
        })

        samples.extend(samps)

    stream_info = {
        "format": format_name or "unknown",
        "total_packets": total_packets,
        "total_samples": len(samples),
        "channels_detected": detected_channels,
        "sample_rate_detected": detected_rate,
        "seq_gaps": seq_gaps,
        "duration": (len(samples) / sample_rate) if samples else 0,
    }

    return samples, packet_info, stream_info


# AAF sample rate field encoding (IEEE 1722-2016 Table 7)
AAF_SAMPLE_RATES = {
    0x01: 8000,
    0x02: 16000,
    0x03: 32000,
    0x04: 44100,
    0x05: 48000,
    0x06: 88200,
    0x07: 96000,
    0x08: 176400,
    0x09: 192000,
}


def _parse_aaf_packet(raw, offset, channel, bit_depth):
    """
    Parse AAF PCM AVTP packet.
    AAF header is 24 bytes from subtype:
      [0]    subtype
      [1]    sv/version/mr/tv flags
      [2]    seq_num
      [3]    tu/reserved
      [4:12] stream_id (8 bytes)
      [12:16] avtp_timestamp
      [16]   format
      [17]   nsr(4) | reserved(2) | padding(2)
      [18]   channels_per_frame
      [19]   bit_depth
      [20:22] stream_data_length
      [22]   evt(4) | sparse(1) | reserved(3)
      [23]   reserved
      [24:]  PCM sample data (big-endian, 32-bit containers, left-justified)
    """
    hdr_len = 24
    if len(raw) < offset + hdr_len:
        return None

    nsr_byte = raw[offset + 17]
    nsr = (nsr_byte >> 4) & 0x0F
    nch = raw[offset + 18]
    bd = raw[offset + 19]
    sdl = struct.unpack("!H", raw[offset + 20:offset + 22])[0]

    if nch == 0:
        return None

    sr = AAF_SAMPLE_RATES.get(nsr, 48000)

    data_start = offset + hdr_len
    data_end = min(data_start + sdl, len(raw))
    data = raw[data_start:data_end]

    # Samples are in 32-bit big-endian containers, left-justified 24-bit
    # For bit_depth <= 24 in 32-bit container: top 24 bits are the sample
    container_size = 4  # 32-bit
    samples_per_frame = len(data) // (nch * container_size)

    extracted = []
    for s in range(samples_per_frame):
        for ch in range(nch):
            idx = (s * nch + ch) * container_size
            if idx + 3 > len(data):
                break
            if ch == channel:
                # Big-endian 24-bit left-justified in 32-bit: bytes [0]=MSB, [1]=MID, [2]=LSB
                val = (data[idx] << 16) | (data[idx + 1] << 8) | data[idx + 2]
                # Sign extend from 24-bit
                if val & 0x800000:
                    val -= 0x1000000
                extracted.append(val)

    return extracted, nch, sr


def _parse_61883_packet(raw, offset, channel):
    """
    Parse IEC 61883-6 AM824 AVTP packet.
    61883 AVTP header (24 bytes):
      [0]    subtype (0x00)
      [1]    flags (sv, version, mr, tv)
      [2]    seq_num
      [3]    tu/reserved
      [4:12] stream_id (8 bytes)
      [12:16] avtp_timestamp
      [16:20] gateway_info
      [20:22] stream_data_length
      [22]   tag(2) | channel(6)
      [23]   tcode(4) | sy(4)
      [24:]  stream_data: 8-byte CIP header then AM824 quadlets

    CIP header (8 bytes):
      [0]    00 | SID(6)
      [1]    DBS
      [2]    FN(2) | QPC(3) | SPH(1) | reserved(2)
      [3]    DBC
      [4]    00 | FMT(6)
      [5]    FDF (SFC bits for audio)
      [6:8]  SYT

    AM824 quadlets: [label(8)][sample_MSB(8)][sample_MID(8)][sample_LSB(8)]
    Label 0x40 = MBLA (multi-bit linear audio)
    """
    hdr_len = 24
    if len(raw) < offset + hdr_len:
        return None

    sdl = struct.unpack("!H", raw[offset + 20:offset + 22])[0]
    data_start = offset + hdr_len
    data_end = min(data_start + sdl, len(raw))
    data = raw[data_start:data_end]

    if len(data) < CIP_HEADER_LEN:
        return None

    dbs = data[1]  # data block size (= channels for AM824 audio)
    if dbs == 0:
        return None

    nch = dbs
    quadlet_data = data[CIP_HEADER_LEN:]

    # Each data block = nch quadlets, each quadlet = 4 bytes
    block_size = nch * 4
    if block_size == 0:
        return None

    num_blocks = len(quadlet_data) // block_size

    extracted = []
    for blk in range(num_blocks):
        for ch in range(nch):
            qi = (blk * nch + ch) * 4
            if qi + 4 > len(quadlet_data):
                break
            label = quadlet_data[qi]
            if ch == channel and label == AM824_LABEL_MBLA:
                val = (quadlet_data[qi + 1] << 16) | (quadlet_data[qi + 2] << 8) | quadlet_data[qi + 3]
                if val & 0x800000:
                    val -= 0x1000000
                extracted.append(val)

    return extracted, nch


# ---------------------------------------------------------------------------
# WAV file output
# ---------------------------------------------------------------------------

def write_wav(samples, path, sample_rate=48000, bit_depth=24):
    """Write samples as a WAV file (mono, 24-bit or 16-bit)."""
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        if bit_depth > 16:
            wf.setsampwidth(3)  # 24-bit
        else:
            wf.setsampwidth(2)
        wf.setframerate(sample_rate)

        if bit_depth > 16:
            # 24-bit little-endian packed
            buf = bytearray(len(samples) * 3)
            for i, s in enumerate(samples):
                # Clamp to 24-bit range
                if s < -0x800000:
                    s = -0x800000
                elif s > 0x7FFFFF:
                    s = 0x7FFFFF
                us = s & 0xFFFFFF
                buf[i * 3] = us & 0xFF
                buf[i * 3 + 1] = (us >> 8) & 0xFF
                buf[i * 3 + 2] = (us >> 16) & 0xFF
            wf.writeframes(bytes(buf))
        else:
            # 16-bit: shift down from 24-bit
            buf = bytearray(len(samples) * 2)
            for i, s in enumerate(samples):
                s16 = max(-32768, min(32767, s >> 8))
                buf[i * 2] = s16 & 0xFF
                buf[i * 2 + 1] = (s16 >> 8) & 0xFF
            wf.writeframes(bytes(buf))

    print(f"WAV written: {path} ({len(samples)} samples, {sample_rate} Hz, {bit_depth}-bit)")


# ---------------------------------------------------------------------------
# Audio analysis (numpy path)
# ---------------------------------------------------------------------------

def _rms(arr):
    """RMS of a numpy array."""
    return np.sqrt(np.mean(arr.astype(np.float64) ** 2))


def analyze_audio_numpy(samples, sample_rate, bit_depth):
    """Full analysis using numpy."""
    results = {}
    data = np.array(samples, dtype=np.float64)
    n = len(data)
    if n == 0:
        return {"error": "No samples to analyze"}

    max_val = (1 << (bit_depth - 1)) - 1
    duration = n / sample_rate

    results["total_samples"] = n
    results["duration_sec"] = duration
    results["peak"] = int(np.max(np.abs(data)))
    results["peak_dbfs"] = 20 * math.log10(max(np.max(np.abs(data)), 1) / max_val)
    results["rms"] = float(_rms(data))
    results["rms_dbfs"] = 20 * math.log10(max(results["rms"], 1) / max_val)

    # --- Discontinuity detection ---
    diffs = np.abs(np.diff(data))
    # Adaptive threshold: use median of diffs + a large multiplier
    median_diff = np.median(diffs)
    if median_diff < 1:
        median_diff = 1
    disc_threshold = max(median_diff * 20, max_val * 0.01)
    disc_indices = np.where(diffs > disc_threshold)[0]

    disc_times = disc_indices / sample_rate
    results["discontinuities"] = {
        "count": len(disc_indices),
        "threshold": disc_threshold,
        "timestamps": disc_times.tolist()[:50],  # first 50
    }

    # --- Silence gap detection ---
    # A "silence gap" is a run of near-zero samples inside a non-silent stream
    silence_threshold = max_val * 0.001  # -60 dBFS
    is_silent = np.abs(data) < silence_threshold
    # Find runs of silence longer than 1ms
    min_gap_samples = max(int(sample_rate * 0.001), 1)
    silence_gaps = []
    in_gap = False
    gap_start = 0
    for i in range(n):
        if is_silent[i]:
            if not in_gap:
                in_gap = True
                gap_start = i
        else:
            if in_gap:
                gap_len = i - gap_start
                if gap_len >= min_gap_samples:
                    silence_gaps.append({
                        "start_time": gap_start / sample_rate,
                        "duration_ms": gap_len / sample_rate * 1000,
                        "start_sample": int(gap_start),
                    })
                in_gap = False
    # Handle trailing gap
    if in_gap:
        gap_len = n - gap_start
        if gap_len >= min_gap_samples:
            silence_gaps.append({
                "start_time": gap_start / sample_rate,
                "duration_ms": gap_len / sample_rate * 1000,
                "start_sample": int(gap_start),
            })

    # Exclude leading/trailing silence from gap report
    # Find first/last non-silent sample
    non_silent = np.where(~is_silent)[0]
    if len(non_silent) > 0:
        content_start = non_silent[0]
        content_end = non_silent[-1]
        silence_gaps = [g for g in silence_gaps
                        if g["start_sample"] > content_start
                        and g["start_sample"] < content_end]

    results["silence_gaps"] = {
        "count": len(silence_gaps),
        "gaps": silence_gaps[:50],
    }

    # --- RMS envelope for periodic artifact detection ---
    window_samples = max(int(sample_rate * 0.001), 1)  # 1ms window
    n_windows = n // window_samples
    if n_windows > 0:
        trimmed = data[:n_windows * window_samples].reshape(n_windows, window_samples)
        envelope = np.sqrt(np.mean(trimmed.astype(np.float64) ** 2, axis=1))

        # Detect spikes in envelope (> 3x median or < 0.3x median for dips)
        env_median = np.median(envelope)
        if env_median > 0:
            env_ratio = envelope / env_median
            spike_idx = np.where((env_ratio > 3.0) | (env_ratio < 0.3))[0]
            spike_times = spike_idx * window_samples / sample_rate

            # Look for periodicity in spike times
            periodic_info = _detect_periodicity(spike_times)
            results["envelope_anomalies"] = {
                "count": len(spike_idx),
                "timestamps": spike_times.tolist()[:50],
                "periodic": periodic_info,
            }
        else:
            results["envelope_anomalies"] = {
                "count": 0,
                "timestamps": [],
                "periodic": None,
            }

    # --- FFT / frequency analysis ---
    if n >= 1024:
        fft_results = _fft_analysis(data, sample_rate, max_val)
        results.update(fft_results)

    # --- Phase continuity for sine wave ---
    if results.get("fundamental_freq") and results["fundamental_freq"] > 10:
        phase_results = _phase_analysis(data, sample_rate, results["fundamental_freq"])
        results["phase_analysis"] = phase_results

    return results


def _detect_periodicity(spike_times):
    """Check if spike timestamps have a regular interval."""
    if len(spike_times) < 3:
        return None

    intervals = np.diff(spike_times)
    if len(intervals) < 2:
        return None

    # Look for a dominant interval
    median_interval = float(np.median(intervals))
    if median_interval < 0.01:
        return None

    # Check how many intervals are close to the median (within 20%)
    tolerance = median_interval * 0.2
    matching = np.sum(np.abs(intervals - median_interval) < tolerance)
    fraction = matching / len(intervals)

    if fraction > 0.5 and len(intervals) >= 2:
        return {
            "interval_sec": round(median_interval, 4),
            "interval_ms": round(median_interval * 1000, 1),
            "consistency": round(float(fraction), 3),
            "count": int(matching),
            "description": f"Periodic artifacts every ~{median_interval*1000:.0f} ms "
                           f"({fraction*100:.0f}% of {len(intervals)} intervals match)",
        }
    return None


def _fft_analysis(data, sample_rate, max_val):
    """FFT-based frequency, SNR, and THD analysis."""
    results = {}

    # Use a window to reduce spectral leakage
    n = len(data)
    # Use power-of-2 FFT length for efficiency, up to 2^18
    fft_len = min(2 ** int(math.log2(n)), 1 << 18)
    segment = data[:fft_len]
    window = np.hanning(fft_len)
    windowed = segment * window

    spectrum = np.fft.rfft(windowed)
    magnitude = np.abs(spectrum) / fft_len
    freqs = np.fft.rfftfreq(fft_len, 1.0 / sample_rate)

    # Skip DC component
    magnitude[0] = 0

    # Find fundamental frequency (strongest peak)
    fund_idx = np.argmax(magnitude)
    fund_freq = float(freqs[fund_idx])
    fund_mag = float(magnitude[fund_idx])

    results["fundamental_freq"] = round(fund_freq, 2)
    results["fundamental_mag_dbfs"] = round(
        20 * math.log10(max(fund_mag * 2, 1e-20) / max_val), 2)

    # SNR: signal power at fundamental vs everything else
    # Define fundamental bin range (+/- 5 bins)
    bin_margin = 5
    fund_lo = max(1, fund_idx - bin_margin)
    fund_hi = min(len(magnitude) - 1, fund_idx + bin_margin)

    signal_power = np.sum(magnitude[fund_lo:fund_hi + 1] ** 2)
    total_power = np.sum(magnitude[1:] ** 2)
    noise_power = total_power - signal_power

    if noise_power > 0 and signal_power > 0:
        results["snr_db"] = round(10 * math.log10(signal_power / noise_power), 2)
    else:
        results["snr_db"] = float("inf")

    # THD: sum of harmonic magnitudes vs fundamental
    if fund_freq > 10:
        harmonic_power = 0.0
        harmonics_found = []
        for h in range(2, 11):  # 2nd through 10th harmonic
            h_freq = fund_freq * h
            if h_freq >= sample_rate / 2:
                break
            h_idx = int(round(h_freq / (sample_rate / fft_len)))
            if h_idx < len(magnitude):
                h_lo = max(1, h_idx - 2)
                h_hi = min(len(magnitude) - 1, h_idx + 2)
                h_mag = float(np.max(magnitude[h_lo:h_hi + 1]))
                harmonic_power += h_mag ** 2
                if h_mag > fund_mag * 0.001:
                    harmonics_found.append({
                        "harmonic": h,
                        "freq": round(h_freq, 1),
                        "level_db": round(20 * math.log10(max(h_mag, 1e-20) / max(fund_mag, 1e-20)), 2),
                    })

        if fund_mag > 0:
            thd = math.sqrt(harmonic_power) / fund_mag * 100
            results["thd_percent"] = round(thd, 4)
            results["thd_db"] = round(20 * math.log10(max(thd / 100, 1e-20)), 2)
        else:
            results["thd_percent"] = 0.0
            results["thd_db"] = float("-inf")

        results["harmonics"] = harmonics_found

    return results


def _phase_analysis(data, sample_rate, freq):
    """Analyze phase continuity of a sine wave using the analytic signal."""
    results = {}

    try:
        from scipy.signal import hilbert
        analytic = hilbert(data)
    except ImportError:
        # Manual analytic signal via FFT
        n = len(data)
        spectrum = np.fft.fft(data)
        h = np.zeros(n)
        if n % 2 == 0:
            h[0] = 1
            h[1:n // 2] = 2
            h[n // 2] = 1
        else:
            h[0] = 1
            h[1:(n + 1) // 2] = 2
        analytic = np.fft.ifft(spectrum * h)

    inst_phase = np.unwrap(np.angle(analytic))

    # Expected phase progression per sample
    phase_per_sample = 2 * math.pi * freq / sample_rate
    expected_phase = np.arange(len(data)) * phase_per_sample

    # Remove linear trend (the expected progression) to get phase error
    # Fit to account for initial phase offset
    phase_detrended = inst_phase - expected_phase
    # Remove constant offset
    phase_detrended -= np.mean(phase_detrended)

    # Phase discontinuities: large jumps in detrended phase
    phase_diffs = np.abs(np.diff(phase_detrended))
    # Threshold: anything > 0.1 radians is suspect
    phase_disc_idx = np.where(phase_diffs > 0.1)[0]
    phase_disc_times = phase_disc_idx / sample_rate

    results["phase_discontinuities"] = len(phase_disc_idx)
    results["phase_disc_timestamps"] = phase_disc_times.tolist()[:50]
    results["max_phase_error_rad"] = round(float(np.max(np.abs(phase_detrended))), 4)
    results["rms_phase_error_rad"] = round(float(np.sqrt(np.mean(phase_detrended ** 2))), 6)

    # Check for periodic phase discontinuities
    if len(phase_disc_times) >= 3:
        periodic = _detect_periodicity(phase_disc_times)
        results["periodic_phase_glitch"] = periodic

    return results


# ---------------------------------------------------------------------------
# Audio analysis (pure Python fallback)
# ---------------------------------------------------------------------------

def analyze_audio_basic(samples, sample_rate, bit_depth):
    """Basic analysis without numpy."""
    results = {}
    n = len(samples)
    if n == 0:
        return {"error": "No samples to analyze"}

    max_val = (1 << (bit_depth - 1)) - 1
    duration = n / sample_rate

    peak = max(abs(s) for s in samples)
    rms_val = math.sqrt(sum(s * s for s in samples) / n)

    results["total_samples"] = n
    results["duration_sec"] = duration
    results["peak"] = peak
    results["peak_dbfs"] = 20 * math.log10(max(peak, 1) / max_val)
    results["rms"] = rms_val
    results["rms_dbfs"] = 20 * math.log10(max(rms_val, 1) / max_val)

    # Discontinuity detection
    diffs = [abs(samples[i + 1] - samples[i]) for i in range(n - 1)]
    diffs_sorted = sorted(diffs)
    median_diff = diffs_sorted[len(diffs_sorted) // 2] if diffs_sorted else 0
    if median_diff < 1:
        median_diff = 1
    disc_threshold = max(median_diff * 20, max_val * 0.01)

    disc_times = []
    for i, d in enumerate(diffs):
        if d > disc_threshold:
            disc_times.append(i / sample_rate)
            if len(disc_times) >= 50:
                break

    disc_count = sum(1 for d in diffs if d > disc_threshold)
    results["discontinuities"] = {
        "count": disc_count,
        "threshold": disc_threshold,
        "timestamps": disc_times,
    }

    # Silence gap detection
    silence_threshold = max_val * 0.001
    min_gap_samples = max(int(sample_rate * 0.001), 1)
    silence_gaps = []
    in_gap = False
    gap_start = 0

    # Find content boundaries
    content_start = 0
    content_end = n - 1
    for i in range(n):
        if abs(samples[i]) >= silence_threshold:
            content_start = i
            break
    for i in range(n - 1, -1, -1):
        if abs(samples[i]) >= silence_threshold:
            content_end = i
            break

    for i in range(content_start, content_end + 1):
        if abs(samples[i]) < silence_threshold:
            if not in_gap:
                in_gap = True
                gap_start = i
        else:
            if in_gap:
                gap_len = i - gap_start
                if gap_len >= min_gap_samples:
                    silence_gaps.append({
                        "start_time": gap_start / sample_rate,
                        "duration_ms": gap_len / sample_rate * 1000,
                    })
                in_gap = False

    results["silence_gaps"] = {
        "count": len(silence_gaps),
        "gaps": silence_gaps[:50],
    }

    # Periodic artifact detection from discontinuity timestamps
    if len(disc_times) >= 3:
        intervals = [disc_times[i + 1] - disc_times[i] for i in range(len(disc_times) - 1)]
        intervals_sorted = sorted(intervals)
        med_interval = intervals_sorted[len(intervals_sorted) // 2]
        if med_interval > 0.01:
            tolerance = med_interval * 0.2
            matching = sum(1 for iv in intervals if abs(iv - med_interval) < tolerance)
            fraction = matching / len(intervals)
            if fraction > 0.5:
                results["periodic_artifacts"] = {
                    "interval_sec": round(med_interval, 4),
                    "interval_ms": round(med_interval * 1000, 1),
                    "consistency": round(fraction, 3),
                    "description": f"Periodic artifacts every ~{med_interval*1000:.0f} ms "
                                   f"({fraction*100:.0f}% of {len(intervals)} intervals match)",
                }

    results["note"] = "Install numpy for FFT, SNR, THD, and phase analysis"

    return results


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_report(stream_info, analysis, sample_rate):
    """Print a human-readable analysis report."""
    print()
    print("=" * 70)
    print("  AVB AVTP Audio Stream Analysis Report")
    print("=" * 70)

    # Stream info
    print()
    print("Stream Information:")
    print(f"  Format:            {stream_info['format']}")
    print(f"  Total packets:     {stream_info['total_packets']}")
    print(f"  Total samples:     {stream_info['total_samples']}")
    print(f"  Channels detected: {stream_info['channels_detected']}")
    sr = stream_info.get('sample_rate_detected') or sample_rate
    print(f"  Sample rate:       {sr} Hz" +
          (" (detected)" if stream_info.get('sample_rate_detected') else " (assumed)"))
    print(f"  Duration:          {stream_info['duration']:.3f} sec")
    print(f"  Sequence gaps:     {stream_info['seq_gaps']}")

    if "error" in analysis:
        print(f"\n  ERROR: {analysis['error']}")
        return

    # Level info
    print()
    print("Signal Level:")
    print(f"  Peak:              {analysis['peak']} ({analysis['peak_dbfs']:.1f} dBFS)")
    print(f"  RMS:               {analysis['rms']:.1f} ({analysis['rms_dbfs']:.1f} dBFS)")

    # Discontinuities
    disc = analysis.get("discontinuities", {})
    print()
    print("Discontinuity Detection:")
    print(f"  Count:             {disc.get('count', 0)}")
    print(f"  Threshold:         {disc.get('threshold', 0):.0f}")
    if disc.get("timestamps"):
        print(f"  First 10 timestamps:")
        for t in disc["timestamps"][:10]:
            print(f"    {t:.6f} sec")

    # Silence gaps
    gaps = analysis.get("silence_gaps", {})
    print()
    print("Silence Gaps (within content):")
    print(f"  Count:             {gaps.get('count', 0)}")
    if gaps.get("gaps"):
        print(f"  First 10 gaps:")
        for g in gaps["gaps"][:10]:
            print(f"    at {g['start_time']:.6f} sec, duration {g['duration_ms']:.2f} ms")

    # Envelope anomalies
    env = analysis.get("envelope_anomalies", {})
    if env:
        print()
        print("RMS Envelope Anomalies (1 ms window):")
        print(f"  Count:             {env.get('count', 0)}")
        if env.get("timestamps"):
            print(f"  First 10 timestamps:")
            for t in env["timestamps"][:10]:
                print(f"    {t:.6f} sec")
        periodic = env.get("periodic")
        if periodic:
            print(f"  ** PERIODIC: {periodic['description']}")

    # Frequency / FFT
    if "fundamental_freq" in analysis:
        print()
        print("Frequency Analysis (FFT):")
        print(f"  Fundamental freq:  {analysis['fundamental_freq']} Hz")
        print(f"  Fundamental level: {analysis['fundamental_mag_dbfs']} dBFS")
        if "snr_db" in analysis:
            snr = analysis["snr_db"]
            print(f"  SNR:               {snr:.1f} dB" if snr < 200 else f"  SNR:               >200 dB")
        if "thd_percent" in analysis:
            print(f"  THD:               {analysis['thd_percent']:.4f}% ({analysis['thd_db']:.1f} dB)")
        if analysis.get("harmonics"):
            print(f"  Significant harmonics:")
            for h in analysis["harmonics"]:
                print(f"    {h['harmonic']}x ({h['freq']:.0f} Hz): {h['level_db']:.1f} dB rel")

    # Phase analysis
    phase = analysis.get("phase_analysis", {})
    if phase:
        print()
        print("Phase Continuity (sine wave):")
        print(f"  Phase discontinuities: {phase.get('phase_discontinuities', 0)}")
        print(f"  Max phase error:       {phase.get('max_phase_error_rad', 0):.4f} rad")
        print(f"  RMS phase error:       {phase.get('rms_phase_error_rad', 0):.6f} rad")
        if phase.get("periodic_phase_glitch"):
            pg = phase["periodic_phase_glitch"]
            print(f"  ** PERIODIC PHASE GLITCH: {pg['description']}")
        if phase.get("phase_disc_timestamps"):
            print(f"  First 10 discontinuity timestamps:")
            for t in phase["phase_disc_timestamps"][:10]:
                print(f"    {t:.6f} sec")

    # Periodic artifacts (basic mode)
    pa = analysis.get("periodic_artifacts")
    if pa:
        print()
        print(f"  ** PERIODIC ARTIFACTS: {pa['description']}")

    if analysis.get("note"):
        print()
        print(f"  Note: {analysis['note']}")

    print()
    print("=" * 70)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Extract and analyze PCM audio from AVTP packet captures.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("pcap", help="Input pcap or pcapng file")
    parser.add_argument("--wav", metavar="FILE", help="Write extracted audio to WAV file")
    parser.add_argument("--src-mac", metavar="MAC",
                        help="Filter by source MAC address (e.g., 00:11:22:33:44:55)")
    parser.add_argument("--channel", type=int, default=0,
                        help="Channel index to extract (default: 0)")
    parser.add_argument("--sample-rate", type=int, default=48000,
                        help="Assumed sample rate in Hz (default: 48000)")
    parser.add_argument("--bit-depth", type=int, default=24,
                        help="Assumed bit depth (default: 24)")
    args = parser.parse_args()

    if not os.path.isfile(args.pcap):
        print(f"ERROR: File not found: {args.pcap}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {args.pcap} ...")
    samples, packet_info, stream_info = extract_avtp_audio(
        args.pcap,
        src_mac_filter=args.src_mac,
        channel=args.channel,
        sample_rate=args.sample_rate,
        bit_depth=args.bit_depth,
    )

    if not samples:
        print("No AVTP audio packets found in capture.", file=sys.stderr)
        if args.src_mac:
            print(f"(filtered by source MAC: {args.src_mac})", file=sys.stderr)
        sys.exit(1)

    # Use detected sample rate if available, otherwise use argument
    sample_rate = stream_info.get("sample_rate_detected") or args.sample_rate
    bit_depth = args.bit_depth

    print(f"Extracted {len(samples)} samples from {stream_info['total_packets']} packets "
          f"({stream_info['format']})")

    # Write WAV if requested
    if args.wav:
        write_wav(samples, args.wav, sample_rate, bit_depth)

    # Analyze
    print("Analyzing audio ...")
    if HAS_NUMPY:
        analysis = analyze_audio_numpy(samples, sample_rate, bit_depth)
    else:
        analysis = analyze_audio_basic(samples, sample_rate, bit_depth)

    print_report(stream_info, analysis, sample_rate)


if __name__ == "__main__":
    main()
