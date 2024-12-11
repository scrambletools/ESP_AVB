# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS utils.c network.c gptp.c avtp.c atdecc.c talker.c main.c # list the source files of this component
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES REQUIRES freertos esp_system nvs_flash driver esp_timer lwip esp_netif esp_event esp_eth  # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)
