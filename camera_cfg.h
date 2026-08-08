#pragma once
#include "esp_camera.h"

// =====================================================================
//  WATT City — XIAO ESP32S3 Sense camera geometry config
//
//  SINGLE SOURCE OF TRUTH. Both the calibration-capture sketch and the
//  runtime AprilTag firmware must include this file and call
//  wcCameraInit() / wcApplyGeometry(). If you change anything in the
//  GEOMETRIC section below, xiao_intrinsics.json is invalid and you
//  must recalibrate.
//
//  Non-geometric settings (jpeg_quality, brightness, saturation, lenc)
//  can differ between calibration and runtime without consequence.
// =====================================================================

// ---------------------- GEOMETRIC (locked) ---------------------------
#define WC_FRAMESIZE   FRAMESIZE_QVGA   // 320x240
#define WC_EXPECT_W    320
#define WC_EXPECT_H    240
#define WC_HMIRROR     0                // mirroring moves cx. Must match.
#define WC_VFLIP       0                // flipping moves cy. Must match.
#define WC_DCW         1                // downsize/crop window: changes FOV
// ---------------------------------------------------------------------

// Pin map for CAMERA_MODEL_XIAO_ESP32S3. Cross-check against the
// camera_pins.h shipped with your esp32-camera version before trusting it.
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// Apply the geometric settings. Call after esp_camera_init().
inline void wcApplyGeometry(sensor_t *s) {
  s->set_framesize(s, WC_FRAMESIZE);
  s->set_hmirror(s, WC_HMIRROR);
  s->set_vflip(s, WC_VFLIP);
  s->set_dcw(s, WC_DCW);
}

// Lock exposure and white balance so the auto-algorithms can't drift
// mid-capture. aec is a starting point; tune with wcSetExposure().
inline void wcLockExposure(sensor_t *s, int aec_value = 300) {
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_aec2(s, 0);
  s->set_aec_value(s, aec_value);
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 0);
}

// Initialise the camera. pixformat: PIXFORMAT_GRAYSCALE for focusing,
// PIXFORMAT_JPEG for calibration capture. quality 4 = best (calibration),
// 12 = normal (runtime). Returns ESP_OK on success.
inline esp_err_t wcCameraInit(pixformat_t pixformat, int jpeg_quality = 12) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;   c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;   c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;   c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;   c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.frame_size   = WC_FRAMESIZE;
  c.pixel_format = pixformat;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.jpeg_quality = jpeg_quality;
  c.fb_count     = 2;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) return err;

  sensor_t *s = esp_camera_sensor_get();
  wcApplyGeometry(s);
  return ESP_OK;
}

// Grab and discard frames so the sensor pipeline settles. The first few
// frames after init or a framesize change are unreliable.
inline void wcFlushFrames(int n = 10) {
  for (int i = 0; i < n; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(30);
  }
}

// Print the geometric fingerprint and hard-fail on a mismatch.
// Run this at boot in BOTH sketches. If the two ever print different
// lines, your intrinsics do not apply.
inline bool wcVerifyGeometry() {
  sensor_t *s = esp_camera_sensor_get();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[cfg] FAIL: no frame"); return false; }

  int w = fb->width, h = fb->height;
  esp_camera_fb_return(fb);

  Serial.printf("[cfg] actual %dx%d  framesize=%d hmirror=%d vflip=%d  sensor_pid=0x%02X\n",
                w, h, (int)s->status.framesize, (int)s->status.hmirror,
                (int)s->status.vflip, s->id.PID);

  bool ok = true;
  if (w != WC_EXPECT_W || h != WC_EXPECT_H) {
    Serial.printf("[cfg] FAIL: expected %dx%d\n", WC_EXPECT_W, WC_EXPECT_H);
    ok = false;
  }
  if ((int)s->status.hmirror != WC_HMIRROR || (int)s->status.vflip != WC_VFLIP) {
    Serial.println("[cfg] FAIL: mirror/flip mismatch");
    ok = false;
  }
  Serial.println(ok ? "[cfg] geometry OK" : "[cfg] GEOMETRY MISMATCH — do not calibrate");
  return ok;
}
