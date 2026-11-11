# Parked: LR2021 (RadioLib) radio driver

Not built. This is the earlier driver for the Seeed Wio-LR2021 module, kept in case a working module is fitted later.
To use it again: add `devices/radio_lr2021/radio.cpp` and RadioLib 7.8.1 to `atlas_sources.cmake`, use this folder's
`radio.h` instead of `devices/radio/radio.h`, and enable C++ in the root CMakeLists. The ground sketch for it is in
`ground/atlas_rx_lr2021/`.
