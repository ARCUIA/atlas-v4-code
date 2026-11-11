# Your sources, kept out of the CubeMX-regenerated files.
# Included from the root CMakeLists.txt with ONE line (see instructions).

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    app/app_time.c
    app/log_service.c
    app/dump.c
    app/dump_usb.c
    app/core/ringbuf.c
    app/core/logblk.c
    app/core/ubx.c
    app/core/gps_cfg.c
    app/core/pkt.c
    app/core/rylr.c
    app/core/rylr_link.c
    app/core/lora_airtime.c
    app/core/gpsmsg.c
    app/core/telemetry.c
    app/telemetry_sources.c
    app/gps_beacon.c
    app/app_isr.c

    devices/devices.c
    devices/spi_dev.c
    devices/imu/imu.c
    devices/baro/baro.c
    devices/sd/sd_spi.c
    devices/gps/gps.c
    devices/radio/radio.c

    # ST platform-independent drivers (third-party code, never edited)
    third_party/lsm6dsv80x-pid/lsm6dsv80x_reg.c
    third_party/lps22df-pid/lps22df_reg.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    app
    app/core
    devices
    devices/imu
    devices/baro
    devices/sd
    devices/gps
    devices/radio
    third_party/lsm6dsv80x-pid
    third_party/lps22df-pid
)
