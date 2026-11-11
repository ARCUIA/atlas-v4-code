# Ground station

- `teammate_transmitter/Reserve_GPS_Transmitter_Code_Works.ino`: the teammate's transmitter (reference copy): sends `AT+SEND=<addr>,<len>,YG,<lat>,<lon>` or `NG,0,0` once a second.
- `teammate_receiver/Reserve_GPS_Reciever_Code_Works.ino`: the receiver in use (a copy, unchanged). It reads the RYLR998's
  serial output and expects the payload `YG,<lat>,<lon>` or `NG`. It never configures its module, so the module's settings
  are whatever it already holds (reported: network 6, address 1, 915 MHz; RF parameters at factory defaults unless changed).
- `rylr_settings_reader/`: prints the receiver module's current settings so `devices/radio/radio.h` can be matched to them.
- `parked/atlas_rx_rylr_hex/`: an earlier receiver of mine (binary packets as hex, RGB LCD). Not needed for this setup.
- `parked/atlas_rx_lr2021/`: the receiver for the LR2021 module that was given up on.
