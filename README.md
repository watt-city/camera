# WATT City: Camera

Vision code for the WATT City car's camera co-processor: a **Seeed XIAO ESP32S3 Sense** (OV2640) that detects **AprilTags (36h11)** at intersections and reports them to the car's main board over UART. Also includes the tools used to focus and calibrate the camera.

**Owned by:** Camera team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`autonomous-car`](https://github.com/watt-city/autonomous-car), [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight).

---

## Repo layout

```
firmware/                         ← Arduino sketches (each in a folder matching its name)
  xiao_apriltag_largest/          ← CURRENT: detect largest tag, print TAG lines over UART + hit-rate stats
  vision_node_watt_link/          ← July version: sends the binary packet the car's main_board expects
  focus_meter/                    ← live focus score, for setting the lens
  calib_capture/                  ← serves calibration JPEGs over WiFi (needs secrets.h)
  shared/camera_cfg.h             ← THE camera geometry config. Edit only this copy.
calibration/                      ← Python tools (run on your laptop)
  calibrate_xiao.py               ← fits camera intrinsics from checkerboard photos
  check_pose.py                   ← live pose readout to sanity-check the intrinsics
  calibration_target_a4.pdf       ← printable 9×6 inner-corner checkerboard
  requirements.txt
tools/sync_camera_cfg.py          ← copies shared/camera_cfg.h into each sketch that uses it
```

## Setup

**Arduino IDE**
1. Install the **esp32 by Espressif** board package.
2. Install [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32): download the ZIP, then *Sketch → Include Library → Add .ZIP Library*.
3. Board settings: **XIAO_ESP32S3**, **PSRAM: OPI PSRAM (required; the camera won't start without it)**, Flash 8MB, CPU 240 MHz.

**Python** (for calibration)
```bash
cd calibration
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Using the sketches

### `xiao_apriltag_largest`: tag detection (current)
- Detects 36h11 tags, picks the **largest in frame** (the nearest), and writes one line per frame on **Serial1 at 115200** (TX = GPIO43 / D6, RX = GPIO44 / D7):
  ```
  TAG,<id>,<center_x>,<center_y>,<side_px>,<decision_margin>[,<z_m>]
  TAG,-1                       ← no tag this frame
  ```
  `z_m` only appears if you set `ENABLE_POSE 1` and fill in the `CAL_*` intrinsics.
- USB Serial prints a bench-test summary every 30 frames (hit rate, average tag size, ms/frame). Use it for range testing.
- Tune `FRAME_SIZE` and `QUAD_DECIMATE` together (comments in the sketch). Weak decodes below `MIN_DECISION_MARGIN` are rejected.

### `focus_meter`: set the lens
Open **Tools → Serial Plotter** at 115200, point at a fixed detailed target, and turn the lens until `focus` peaks. `r` resets the peak, `+`/`-` changes exposure, and `p` prints values for the log.

### Calibration workflow
1. Print `calibration/calibration_target_a4.pdf` **at 100% / Actual Size**. Measure a printed square with calipers, since printers scale.
2. In `firmware/calib_capture/`, copy `secrets.example.h` → `secrets.h` and fill in your WiFi (2.4 GHz only). `secrets.h` is gitignored; never commit it.
3. Flash `calib_capture`, note the IP it prints, and save frames from `http://<ip>/capture` into `calibration/captures/` (20+ images, varied angles, the whole board visible).
4. Fit the intrinsics:
   ```bash
   python calibrate_xiao.py "captures/*.jpg" --cols 9 --rows 6 --square <measured mm>
   ```
   This writes `xiao_intrinsics.json`.
5. Sanity-check live:
   ```bash
   python check_pose.py --url http://<ip>/capture --mode tag --tag-size 37 --expect <tape-measured distance in mm>
   ```
6. Put captured images in the **shared folder**, not git. Commit `xiao_intrinsics.json` and record the result in the hardware log.

**Rule:** intrinsics are only valid for the resolution and settings in `camera_cfg.h`'s *GEOMETRIC* section. Change any of those and you must recalibrate.

## Editing `camera_cfg.h`

Arduino can only include headers from the sketch's own folder, so `calib_capture/` and `focus_meter/` each hold a **copy**. Edit **only** `firmware/shared/camera_cfg.h`, then run:

```bash
python tools/sync_camera_cfg.py          # update the copies
python tools/sync_camera_cfg.py --check  # verify (run before opening a PR)
```

## Known issues / open decisions

- **UART format mismatch with the car.** `xiao_apriltag_largest` sends text `TAG,...` lines, but the car's `main_board` firmware expects the binary `VisionPacket` from `watt_link.h` (`[0xAA][0x55][type][len][payload][checksum]`). `vision_node_watt_link` speaks the binary format but predates the tuning work. **Camera + Car teams: pick one format before integration.**
- **Resolution differs from calibration.** `camera_cfg.h` (used for calibration) is VGA 640×480, while `xiao_apriltag_largest` runs QVGA and doesn't include `camera_cfg.h`. This is fine for reading tag IDs; it matters if you turn on `ENABLE_POSE`.
- **`grab_captures.py` is missing.** `calib_capture` mentions a `grab_captures.py` script that was never committed. Save frames manually from `/capture` until someone writes it.
- **Tag size.** The printed tags measure **52 mm** across the outer white edge and **37 mm** across the black square. `check_pose.py --tag-size` asks for the *black square* edge but defaults to 52, so pass `--tag-size 37` for these tags. (`xiao_apriltag_largest` already uses 0.037 m.)

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first push. In short: branch off `main`, open a pull request, get one review, never commit WiFi passwords, and log every test in the hardware log.

## History

`firmware/vision_node_watt_link/` came from the former `watt-city/car-code` repo and was merged here with its commit history.
