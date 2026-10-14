// =====================================================================
// calib_capture.ino — XIAO ESP32S3 Sense (OV2640)
//
// Serves calibration frames over HTTP so grab_captures.py can pull them.
// Uses the SAME camera_cfg.h as the runtime AprilTag firmware, so the
// geometry the intrinsics are fitted to is the geometry you deploy.
//
// SETUP
//   1. Open this folder in the Arduino IDE (camera_cfg.h is already here;
//      it is a copy of firmware/shared/camera_cfg.h)
//   2. Copy secrets.example.h to secrets.h and fill in WIFI_SSID / WIFI_PASS
//   3. Tools > Board          : XIAO_ESP32S3
//      Tools > PSRAM          : OPI PSRAM      <-- required
//   4. Upload, open Serial Monitor @ 115200, note the printed IP
//
// ENDPOINTS
//   GET /capture            single JPEG, QVGA, quality 4
//   GET /health             geometry fingerprint as text
//   GET /exposure?aec=300   set manual exposure (20..1200)
//
// If /health disagrees with the runtime sketch's boot print, your
// intrinsics do not apply to the runtime frames. Fix that first.
// =====================================================================

#include "camera_cfg.h"
#include <WiFi.h>
#include <WebServer.h>

// ---------------------------------------------------------------------
// WiFi credentials live in secrets.h (gitignored -- this repo is public).
// Copy secrets.example.h to secrets.h and fill in your network.
#include "secrets.h"
// ---------------------------------------------------------------------

// Quality 4 = near-lossless. JPEG ringing moves subpixel corners, and a
// corner biased by a third of a pixel is a corner biased by a third of a
// pixel no matter how many images you average. Runtime can use 12.
static const int CALIB_JPEG_QUALITY = 4;

static int aec = 300;

WebServer server(80);

static void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "capture failed");
    return;
  }
  if (fb->format != PIXFORMAT_JPEG) {
    esp_camera_fb_return(fb);
    server.send(500, "text/plain", "frame is not JPEG");
    return;
  }
  server.setContentLength(fb->len);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

static void handleHealth() {
  sensor_t *s = esp_camera_sensor_get();
  camera_fb_t *fb = esp_camera_fb_get();
  int w = fb ? fb->width : -1;
  int h = fb ? fb->height : -1;
  size_t len = fb ? fb->len : 0;
  if (fb) esp_camera_fb_return(fb);

  char buf[320];
  snprintf(buf, sizeof(buf),
           "actual        %dx%d\n"
           "expected      %dx%d\n"
           "framesize     %d\n"
           "hmirror       %d (cfg %d)\n"
           "vflip         %d (cfg %d)\n"
           "dcw           %d (cfg %d)\n"
           "sensor_pid    0x%02X\n"
           "aec_value     %d\n"
           "jpeg_bytes    %u\n",
           w, h, WC_EXPECT_W, WC_EXPECT_H,
           (int)s->status.framesize,
           (int)s->status.hmirror, WC_HMIRROR,
           (int)s->status.vflip, WC_VFLIP,
           (int)s->status.dcw, WC_DCW,
           s->id.PID, aec, (unsigned)len);
  server.send(200, "text/plain", buf);
}

static void handleExposure() {
  if (!server.hasArg("aec")) {
    server.send(400, "text/plain", "usage: /exposure?aec=300");
    return;
  }
  int v = server.arg("aec").toInt();
  if (v < 20) v = 20;
  if (v > 1200) v = 1200;
  aec = v;
  sensor_t *s = esp_camera_sensor_get();
  s->set_aec_value(s, aec);
  wcFlushFrames(3);
  char buf[48];
  snprintf(buf, sizeof(buf), "aec=%d\n", aec);
  server.send(200, "text/plain", buf);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (wcCameraInit(PIXFORMAT_JPEG, CALIB_JPEG_QUALITY) != ESP_OK) {
    Serial.println("camera init failed - check the FPC seating on the B2B connector,");
    Serial.println("and confirm Tools > PSRAM is set to OPI PSRAM");
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  wcLockExposure(s, aec);   // frozen exposure: no AE hunting mid-shoot
  wcFlushFrames(10);

  if (!wcVerifyGeometry()) {
    Serial.println("Refusing to serve frames under mismatched geometry.");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);     // modem sleep adds latency and drops frames
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifi failed - check SSID/PASS, and that the AP is 2.4 GHz");
    while (true) delay(1000);
  }

  server.on("/capture", handleCapture);
  server.on("/health", handleHealth);
  server.on("/exposure", handleExposure);
  server.begin();

  Serial.println();
  Serial.print("ready:  http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");
  Serial.print("health: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/health");
  Serial.println();
  Serial.println("Now run:  python grab_captures.py --url http://<ip>/capture --cols 9 --rows 6");
}

void loop() {
  server.handleClient();
}
