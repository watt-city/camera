// =====================================================================
//  focus_meter.ino — XIAO ESP32S3 Sense (OV2640)
//
//  Live focus score over Serial. Open Tools > Serial Plotter at 115200
//  and turn the lens barrel until "focus" peaks, then back off past the
//  peak and return to it to confirm you found the true maximum.
//
//  Serial commands:
//    r   reset the peak-hold value
//    +   increase exposure       -   decrease exposure
//    p   print the current numbers as text (for the log)
//
//  Put a fixed, detailed target (printed fine text, or your checkerboard)
//  at your working distance and DO NOT move it while adjusting.
// =====================================================================

#include "camera_cfg.h"

static int  aec = 300;
static float peak = 0.0f;

// Tenengrad focus metric over a centre ROI, normalised by mean intensity
// squared so it is less sensitive to lighting changes.
static void focusScore(const uint8_t *buf, int w, int h,
                       float *scoreOut, float *meanOut) {
  const int rw = w / 2, rh = h / 2;
  const int rx = (w - rw) / 2, ry = (h - rh) / 2;

  double acc = 0.0, sum = 0.0;
  long   n   = 0;

  for (int y = ry + 1; y < ry + rh - 1; y++) {
    const uint8_t *row = buf + (size_t)y * w;
    const uint8_t *up  = row - w;
    const uint8_t *dn  = row + w;
    for (int x = rx + 1; x < rx + rw - 1; x++) {
      int gx = (int)row[x + 1] - (int)row[x - 1];
      int gy = (int)dn[x]      - (int)up[x];
      acc += (double)(gx * gx + gy * gy);
      sum += row[x];
      n++;
    }
  }

  float mean = (n > 0) ? (float)(sum / n) : 0.0f;
  float raw  = (n > 0) ? (float)(acc / n) : 0.0f;
  *meanOut  = mean;
  *scoreOut = (mean > 1.0f) ? (raw / (mean * mean)) * 1000.0f : 0.0f;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // GRAYSCALE gives direct pixel access with no JPEG decode.
  if (wcCameraInit(PIXFORMAT_GRAYSCALE) != ESP_OK) {
    Serial.println("camera init failed — check the FPC seating on the B2B connector");
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  wcLockExposure(s, aec);
  wcFlushFrames(10);
  wcVerifyGeometry();

  Serial.println("\nTurn the lens slowly. Watch 'focus' in the Serial Plotter.");
  Serial.println("Aim for mean between 100 and 150 — use + / - if it is outside that.\n");
}

void loop() {
  while (Serial.available()) {
    char cmd = Serial.read();
    sensor_t *s = esp_camera_sensor_get();
    if (cmd == 'r') { peak = 0.0f; }
    else if (cmd == '+') { aec = min(1200, aec + 50); s->set_aec_value(s, aec); }
    else if (cmd == '-') { aec = max(20,   aec - 50); s->set_aec_value(s, aec); }
    else if (cmd == 'p') { Serial.printf("\n>> peak=%.1f aec=%d\n\n", peak, aec); }
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  float score, mean;
  focusScore(fb->buf, fb->width, fb->height, &score, &mean);
  esp_camera_fb_return(fb);

  if (score > peak) peak = score;

  // Serial Plotter format: three labelled series on one line.
  Serial.printf("focus:%.1f\tpeak:%.1f\tmean:%.1f\n", score, peak, mean);
  delay(100);
}
