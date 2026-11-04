# WATT City: Camera

Vision code for the WATT City car's camera co-processor: a **Seeed XIAO ESP32S3 Sense** (OV2640) that detects **AprilTags (36h11)** at intersections and reports them to the car's main board over UART.

**Owned by:** Camera team · **Leads:** Alisa Zhang, Katie Tao · **Questions:** `#watt-city` on WATT Slack

Part of [WATT City](https://github.com/watt-city), IEEE WATT's year-long project at UCLA. Related repos: [`autonomous-car`](https://github.com/watt-city/autonomous-car), [`autonomous-stoplight`](https://github.com/watt-city/autonomous-stoplight).

---

## Repo layout

```
firmware/
  xiao_apriltag_largest/   ← The AprilTag detector. Flash this to the XIAO.
```

## Setup

1. Install the **esp32 by Espressif** board package.
2. Install [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32): download the ZIP, then *Sketch → Include Library → Add .ZIP Library*.
3. Board settings: **XIAO_ESP32S3**, **PSRAM: OPI PSRAM (required; the camera won't start without it)**, Flash 8MB, CPU 240 MHz.
4. Print 36h11 tags with **IDs 0–34 only**. This port loads only the first 35 codes to fit in RAM, so higher IDs never decode. Our printed tags (IDs 0–5) are in [`autonomous-car/docs/`](https://github.com/watt-city/autonomous-car/tree/main/docs): 52 mm outer white edge, **37 mm black square**.

## `xiao_apriltag_largest`: how it works

Every frame it grabs a grayscale image, runs the AprilTag detector, drops weak decodes (decision margin < `MIN_DECISION_MARGIN` = 30), keeps the **largest tag in view** (the nearest), and sends one line on **Serial1 at 115200** (TX = GPIO43 / D6, RX = GPIO44 / D7):

```
TAG,<id>,<center_x>,<center_y>,<side_px>,<decision_margin>
TAG,-1                       ← no tag this frame
```

USB Serial (115200) prints a bench summary every 30 frames: hit rate, tag ID, average tag size in pixels, average decision margin, and ms per frame. Use it for range testing: hold a tag at a few distances and read the hit rate.

Current settings: `FRAME_SIZE` QVGA (320×240), `QUAD_DECIMATE` 2.0, 1-bit error correction, vflip on, auto exposure and auto gain. Change `FRAME_SIZE` and `QUAD_DECIMATE` together (see the comment block at the top of the sketch). Detection runs in its own FreeRTOS task on core 1 with a 32 KB stack, because the default `loop()` stack is too small.

Pose/range output (`ENABLE_POSE`) is off. It needs camera intrinsics from a calibration we don't currently use; reading tag IDs doesn't need them.

## Talking to the car

The car reads these `TAG,...` text lines directly. The format and the car's parser (`watt_parse_tag_line()`) live in [`autonomous-car/firmware/main_board/watt_link.h`](https://github.com/watt-city/autonomous-car/blob/main/firmware/main_board/watt_link.h). **If you change the line format, change that parser too.**

Wiring: XIAO D6 (TX) → car main board RX, plus a common GND.

## History

Calibration tools (checkerboard capture server, focus meter, OpenCV calibration and pose-check scripts) were removed in Nov 2026 because the working detector doesn't use them. They're in git history before that commit if anyone needs pose later.

An older detector, `vision_node_watt_link` (July, from the former `car-code` repo), sent a binary packet to the car. It was removed in Nov 2026 when the team chose the text format; it's in git history.

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first push. In short: pull before you start, push when you finish; push straight to `main` for docs; **use a branch + pull request for anything in `firmware/`**; never commit WiFi passwords; log every test in the hardware log.
