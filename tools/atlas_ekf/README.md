# Atlas v4 EKF + live viewer

* `run_offline.py`  error-state EKF over logged flight CSVs -> states, plots
* `atlas_live.py`   **live Qt viewer**: Attitude (3D) | Raw sensors | GPS + altitude | Log
* `viewer.py`       matplotlib playback of an EKF run (offline)
* `stream.py`       text-stream parser, formatters, serial / replay / file sources
* `firmware/`       `usb_stream.c/.h`: what the board should print (host-tested)
* `eskf.py`, `flightdata.py`, `test_*.py`

## Setup (Zorin / Ubuntu 22)
    sudo apt install python3-venv python3-tk
    python3 -m venv ~/.venvs/atlas && source ~/.venvs/atlas/bin/activate
    pip install numpy pandas matplotlib pyserial PyQt5 pyqtgraph
(`sudo apt install python3-pyqt5` also works if you would rather not pip it; add your user to
`dialout` for the serial port: `sudo usermod -aG dialout $USER`, then log out/in.)

## Live viewer
    python3 atlas_live.py --port auto                      # first STM32 CDC port, bench mode
    python3 atlas_live.py --port /dev/serial/by-id/usb-STMicro...  --flight   # + launch/burnout logic
    cat /dev/ttyACM0 > cap.txt                             # capture, then replay it later:
    python3 atlas_live.py --text-file cap.txt              # (--line-rate N for unstamped lines)
    python3 atlas_live.py --replay flight-1_csv --start 1745 --speed 1 --flight   # fake stream from CSVs
Add `--fake-mag` to a replay to exercise the mag plot with SYNTHETIC data (the CSVs have no mag).

Tabs: **Attitude** 3D rocket + readouts (hold still 5 s after start/"Re-align"; yaw is arbitrary without
the mag). **Raw sensors** accel [g], gyro [deg/s], mag [uT], 3 lines each, selectable window,
peak-preserving downsampling so spikes (ejection charge) are never decimated away.
**GPS + altitude** horizontal track in metres from the first fix, plus baro and GPS height overlaid
(both zeroed at start; "Re-zero alt / GPS" re-references them). **Log** non-sample lines.
The bottom bar always shows your status line, each segment green if it starts `NAME:OK`, red if not.
Keys: Space = pause.

## What the board prints
Keep the 1 Hz status line exactly as it is. Add timestamped sample lines in the same style:

    IMU  t=1234.567890 xl[mg]=-86,-99,988 gy[mdps]=-280,210,140
    MAG  t=1234.567890 mag[mG]=123,-45,300
    BARO t=1234.567890 p[Pa]=98717 T[C]=21.30
    GPS  t=1234.567890 fix=3 sv=9 lat=41.6611234 lon=-91.5301234 hmsl[m]=221.600 vd[mps]=0.100 hacc[m]=0.400 vacc[m]=0.700 sacc[mps]=0.200

`t` = seconds since boot from the same clock as the SD log. Printing the status line alone still works
but gives only host-timestamped IMU/baro and fix/sv: no GPS position, no mag, and the stream rate is
whatever the line rate is, so the viewer labels it "legacy" and the attitude is approximate.

### Firmware integration (`firmware/usb_stream.c`, not wired into your code: I do not have it)
    usb_stream_init();                          /* after MX_USB_DEVICE_Init() */
    usb_stream_imu (t_us, xl_mg, gy_mdps);      /* each IMU sample, from the main loop */
    usb_stream_baro(t_us, p_pa, temp_centi_c);
    usb_stream_mag (t_us, mag_mG);              /* when the LIS2MDL is fitted */
    usb_stream_gps (t_us, fix, numSV, lat, lon, hMSL, velD, hAcc, vAcc, sAcc);   /* NAV-PVT units */
    usb_stream_poll();                          /* every main-loop pass */
    usb_stream_enable(false);                   /* before your binary dump command */
Ring buffer, never blocks, counts dropped lines (`usb_stream_dropped()`, handy in the status line),
fixed-point formatting only (newlib-nano has float printf off). Call the producers from one context.
Budget: ~65 B x 455 Hz = 30 kB/s for IMU, fine for full-speed CDC; decimate the IMU lines if you like,
the EKF just uses whatever rate arrives.
Build check on a PC: `gcc -DUSB_STREAM_HOST_TEST -Ifirmware firmware/usb_stream.c firmware/test_usb_stream.c && ./a.out`

## Offline EKF
    python3 run_offline.py ~/Documents/arc/atlas-v4-code/flight-1_csv      # ~10 s
    python3 viewer.py --offline ~/Documents/arc/atlas-v4-code/flight-1_csv/ekf_out
See the notes in the earlier README section of `eskf.py` docstrings; flight-1 results are in `flight1_ekf_out/`.
Known limits: no accel scale-factor states, GPS vertical off by default (unreliable in flight), yaw
arbitrary without mag, high-g channel unused, 455 vs 480 Hz unresolved.

## Tests
    python3 test_eskf.py     # filter on synthetic flights
    python3 test_stream.py   # parser incl. your exact status line, and C-module round trip (needs gcc)
