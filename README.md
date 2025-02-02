# ESP_AVB

 AVB Implementation using ESP-IDF

 Formats supported by test devices:
 Mac Mini input and output:
 - IEC61883-6: AM8-24, 48/96/192kHz, 8 data blocks non-blocking 8 MBLA
 - IEC61883-6: AM8-24, 48/96/192kHz, 8 data blocks non-blocking synchronous clock 8 MBLA
 - AAF-PCM: 24 bits in 32bit int, 8 channels of 6 samples @ 44.1/48kHz
 - AAF-PCM: 24 bits in 32bit int, 8 channels of 12 samples @ 88.2/96kHz
 - AAF-PCM: 24 bits in 32bit int, 8 channels of 24 samples @ 176.4/192kHz
 Motu 8D input and output:
 - IEC61883-6: AM8-24 (PCM-INT-24), 44.1/48/88.2/96/176.4/192kHz, 1-8 data blocks non-blocking (async) 1-8 MBLA

 Testing notes:
 M1: EID = ...888000
 M1: MAC = ...c7:88
 M1: GM = ...30002

 8D: EID = ...0162e5
 8D: MAC = ...62:e5
 8D: GM = ...50006

 T1: EID = ...230000
 T1: MAC = ...8c:23
 T1: GM = ...00000

Summary of the AVB talker (T), listener (L), controller (C) and bridge (B) interaction:
[Sequence of frame transmissions by each device type]

GPTP (sync with network clock):
TL: GPTP Announce
TL: GPTP Sync
TL: GPTP Follow Up
TL: GPTP Peer Delay Request
B: GPTP Peer Delay Response
B: GPTP Peer Delay Resopnse Follow Up

ATDECC (remote control):
TL: ATDECC ADP Entity Available (resend every ~10sec)
C: ATDECC AECP Read Entity Command
TL: ATDECC AECP Read Entity Response
C: ATDECC ECMP Connect RX Command (4.5sec timeout)
L: ATDECC ECMP Connect RX Response
L: ATDECC ECMP Connect TX Command (2sec timeout)
T: ATDECC ECMP Connect TX Response

MSRP (reserve bandwidth):
T: MVRP VLAN Identifier (resend every ~2sec)
L: MSRP Domain (resend every ~9sec)
T: MSRP Talker Advertise Leave (resend every ~3sec)
B: MSRP Talker Advertise Indicate
L: MVRP VLAN Identifier (resend every ~2sec)
L: MSRP Listener Ready (resend every ~3sec)

AVTP (media streaming):
T: AVTP AAF Stream


PTP/MSRP observed messages:
- apple: pdrq: ? (not recorded)
- motu: pdrq: domain 0, seq 8337, flags 0008, ctrfield other(5), (no response from apple)
- motu: pdrp: domain 0, seq 59006, flags 0208, ctrfield other, rcpt 139415.844269480
- motu: pdrf: domain 0, seq 59006, flags 0208, ctrfield other, orig 139415.844303712
- motu: msrp domain: srclass b, pri 2, vid 2, attr event: joinIn (sent twice?)
- motu: mvrp vlan id: vid 2, attr event: New (sent twice?)
- motu: mvrp vlan id: vid 2, attr event: joinIn
- apple: msrp domain (double): srclass a, pri 3, vid 2, attr event: New
                               srclass b, pri 2, vid 2, attr event: New
- apple: mvrp vlan id: vid 2, attr event: New
- motu: pdrq: domain 0, seq 8338, flags 0008, ctrfield other, (no response from apple)
- apple: pdrq: domain 0, seq 59007, flags 0008, ctrfield other, rcpt 139416.845402112
- motu: pdrp: domain 0, seq 59007, flags 0208, ctrfield other, rcpt 139416.845402112
- motu: pdrf: domain 0, seq 59007, flags 0208, ctrfield other, orig 139416.845434992
    calc unscaledMeanPropagationDelay = -18060 ns
- motu: pdrq: domain 0, seq 8339, flags 0008, ctrfield other
- apple: pdrp: domain 0, seq 8339, flags 0208, ctrfield other, rcpt 263982.626028657
- apple: pdrf: domain 0, seq 8339, flags 0008, ctrfield other, orig 263982.626302120
    calc unscaledMeanPropagationDelay = -25731 ns
- motu: sync: domain 0, seq 360, flags 0208, ctrfield syncmsg(0), period -3
- motu: flup: domain 0, seq 360, flags 0008, ctrfield flwpmsg(2), period -3, prec 139417.412404592, tlv 0
    cannot calc syncRateRatio yet
- motu: sync: domain 0, seq 361, flags 0208, ctrfield syncmsg(0), period -3
- motu: flup: domain 0, seq 361, flags 0008, ctrfield flwpmsg(2), period -3, prec 139417.537406432, tlv 0
    calc syncRateRatio = 1.00209908609909 (-2099 ppm)
- motu: sync: seq 362
- motu: flup: seq 362
- motu: sync: seq 363
- motu: flup: seq 363
- apple: pdrq: seq 59008
- motu: pdrp: seq 59008, rcpt 139417.846058976
- motu: pdrf: seq 59008, orig 139417.846091872
    calc unscaledMeanPropagationDelay = 30552 ns
    calc scaledMeanPropagationDelay = 3.05436311803453e-05 ns
    calc NeighborRateRatio = 1.0005088046969 (-508 ppm)
- motu: sync: seq 364
- motu: flup: seq 364
- motu: sync: seq 365
- motu: flup: seq 365
    calc syncRateRatio = 1.00066856648843 (-668 ppm)
- apple: entavail: seq 59009
- motu: sync: seq 366
- motu: flup: seq 366
    calc syncRateRatio = 0.999278634009081 (721 ppm)
- apple: mvrp vlan id: vid 2, attr event: JoinIn
- motu: announce: domain 0, seq 8238, flags 0008, ctrfield flup(2), period 0
    pri1 246, class 248, acc 0xfe, variance 1, pri2 248
...
- motu: announce: seq 8239
...
- motu: announce: seq 8240
...
- apple: announce: domain 0, seq 333, flags 0008, ctrfield other(5), period 0
    pri1 248, class248, acc 0x21, variance 17258, pri2 235
