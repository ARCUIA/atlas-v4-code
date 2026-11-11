# Atlas v4 EKF prototype

Offline error-state EKF (ESKF) on logged flight CSVs, plus a 3D orientation viewer
that can replay a run or show a live raw-IMU stream over USB.

## Setup (Zorin / Ubuntu 22)
    sudo apt install python3-venv python3-tk
    python3 -m venv ~/.venvs/atlas && source ~/.venvs/atlas/bin/activate
    pip install numpy pandas matplotlib pyserial

## Run
    python3 test_eskf.py                                   # synthetic sanity checks, ~5 s
    python3 run_offline.py ~/Documents/arc/atlas-v4-code/flight-1_csv     # ~10 s for the flight window
    python3 viewer.py --offline ~/Documents/arc/atlas-v4-code/flight-1_csv/ekf_out
    python3 viewer.py --replay  ~/Documents/arc/atlas-v4-code/flight-1_csv --start 1740 --speed 1 --flight
    python3 viewer.py --port /dev/serial/by-id/usb-STMicroelectronics_...   # live, bench mode
Viewer keys: space play/pause, arrows seek (shift = 10 s), up/down speed, `e` next event,
click the right-hand plots to seek; live: `r` re-aligns (hold still for 5 s).

## Files
| file | purpose |
|---|---|
| `eskf.py` | 15-state ESKF (attitude, vel, pos, accel bias, gyro bias) + flight phases. Pure numpy, no I/O. |
| `flightdata.py` | CSV loader, rail-saturation flags, coarse launch finder |
| `run_offline.py` | runs the filter over a window, writes `ekf_states.csv`, `ekf_updates.csv`, `ekf_meta.json`, `ekf_overview.png` |
| `viewer.py` | offline playback / live view / replay-as-live |
| `test_eskf.py` | rotation, alignment, static hold, synthetic flight |

## How it works
* **Frames.** Body = sensor frame. Nav = local NED at the pad. With no magnetometer yaw is arbitrary
  (`--yaw-deg` sets it); horizontal GPS is therefore not used.
* **Alignment.** First `align_s` (5 s) of stationary data: mean specific force gives roll/pitch and the
  nose axis (on flight 1 the nose is **-X**), mean gyro gives the initial bias, mean pressure gives the pad reference.
* **Predict** every IMU sample with the *measured* dt from `t_s`.
* **Pad:** gravity-direction (tilt), zero-velocity and static gyro-bias updates.
* **Launch** = 0.13 s mean of > 2.5 g along the nose; **burnout** = < 0.3 g; **apogee** = filter v_up crosses 0;
  **landed** = 1 s of ~1 g, quiet gyro.
* **Baro** is locked out from launch to burnout + 1 s, and its noise is `sqrt(s0^2 + (lag*|v|)^2)` (lag = 1.5 s),
  because on flight 1 it read 100+ m low just after burnout.
* **Gate** failures relax automatically (`1 + 0.25 n^2`) so a bad stretch cannot lock the filter out forever.
* **Shock.** A sample with any raw axis >= 32760 holds the last good value and inflates accel noise x100
  for 0.5 s (flight 1: ejection charge at t = 1767.4 s, and the event at 1929 s).
* **Chute phase:** tilt (gravity-direction) updates again, because the airframe hangs at ~1 g.

## Flight 1 result (GPS off)
launch 1752.40 s, burnout +1.96 s, apogee detected 1764.8 s (baro pressure minimum 1764.55 s),
peak altitude 871 m (IMU+baro, baro lags so its raw peak is 852 m), landing detected 1931.9 s.
**There is no independent altitude reference in this data, so the 871 m is not validated.** Compare it to
your sim / any other altimeter.

## Known limits / next steps
1. No accel scale-factor or misalignment states. Boost error is covered only by inflated process noise.
2. GPS: only a vertical channel exists and it is off by default. In flight 1 it read ~88 m low at apogee,
   lagged by seconds and dropped to 3-5 SVs during the burn (airborne <4 g model, burn peaks ~14 g).
3. Yaw is unobservable without a mag. GPS-course alignment during boost is the natural next addition.
4. The 455 Hz vs 480 Hz question (polled reads?) is unresolved. Data-ready interrupt + FIFO timestamp is the fix.
5. High-g channel is unused (scale factor unverified).
6. Process noise comes from pad measurements x2-3; retune from flight residuals.

## Live stream protocol (firmware does not emit this yet)
One ASCII line per sample over the USB CDC port, same units as the CSVs:

    I,<t_s>,<ax_mg>,<ay_mg>,<az_mg>,<gx_mdps>,<gy_mdps>,<gz_mdps>
    B,<t_s>,<pressure_pa>                                    (optional)
    G,<t_s>,<fix>,<num_sv>,<hmsl_m>,<vel_d_mps>,<vacc_m>,<sacc_mps>   (optional)

~55 bytes x 455 Hz = 25 kB/s, fine for full-speed CDC. Use a ring buffer and send from the main loop /
TxCplt callback; `CDC_Transmit_FS` returns `USBD_BUSY` if called while a transfer is in flight.
The stream must not collide with the existing dump command (which pauses logging), so give it its own mode.
Rail detection in live mode uses |accel| >= 15.98 g and |gyro| >= 2290 dps because raw counts are not sent.
