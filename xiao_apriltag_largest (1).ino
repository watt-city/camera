// xiao_apriltag_largest.ino
// XIAO ESP32S3 Sense (OV2640) -> tag36h11 detection -> largest tag in frame -> UART out
// Includes 30-frame hit-rate instrumentation for bench range testing.
//
// Board settings (Arduino IDE):
//   Board:        XIAO_ESP32S3
//   PSRAM:        OPI PSRAM        <-- REQUIRED
//   Flash Size:   8MB (64Mb)
//   Partition:    8M with spiffs (or Huge APP)
//   CPU Freq:     240 MHz
//   Core Debug:   None

#include "esp_camera.h"
#include "esp_heap_caps.h"

// raspiduino/apriltag-esp32, installed via Sketch > Include Library > Add .ZIP Library.
// If your copy nests headers differently, open the bundled simple_tag_detect
// example and copy its include lines verbatim.
extern "C" {
  #include "apriltag.h"
  #include "tag36h11.h"
  #include "common/image_u8.h"
  #include "apriltag_pose.h"
}

// =========================================================================
// TUNE THESE TWO TOGETHER
//   Longest range      ->  FRAMESIZE_SVGA (800x600) + 1.0   <- start here
//   Faster, less range ->  FRAMESIZE_VGA  (640x480) + 1.0
//   Fastest, short     ->  FRAMESIZE_QVGA (320x240) + 1.0
// =========================================================================
#define FRAME_SIZE            FRAMESIZE_SVGA
#define QUAD_DECIMATE         1.0f

#define MIN_DECISION_MARGIN   30.0f   // reject weak decodes
#define REPORT_EVERY          30      // frames per bench-test summary line
#define ENABLE_POSE           0       // 1 = also estimate range/pose

#define VISION_UART           Serial1
#define VISION_TX_PIN         43      // XIAO D6
#define VISION_RX_PIN         44      // XIAO D7
#define VISION_BAUD           115200

#if ENABLE_POSE
static const float  CAL_WIDTH  = 640.0f;   // resolution you calibrated at
static const float  CAL_HEIGHT = 480.0f;
static const float  CAL_FX = 0.0f;         // <-- fill in from your calibration
static const float  CAL_FY = 0.0f;
static const float  CAL_CX = 0.0f;
static const float  CAL_CY = 0.0f;
static const float  TAG_SIZE_M = 0.037f;   // BLACK SQUARE edge, metres
#endif

// ---------------------- XIAO ESP32S3 Sense camera pins -------------------
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

static apriltag_family_t   *tf = nullptr;
static apriltag_detector_t *td = nullptr;

// -------------------------------------------------------------------------
static bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;   c.pin_pclk     = PCLK_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;  c.pin_href     = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;   c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn     = PWDN_GPIO_NUM;   c.pin_reset    = RESET_GPIO_NUM;

  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size   = FRAME_SIZE;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 0);    // manual exposure: kills motion blur
  s->set_aec2(s, 0);
  s->set_aec_value(s, 120);      // tune 40..400 for your lighting
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 4);         // tune 0..30
  s->set_contrast(s, 2);
  s->set_brightness(s, 0);
  s->set_whitebal(s, 0);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  return true;
}

static void detectorInit() {
  tf = tag36h11_create();
  td = apriltag_detector_create();
  // NOTE: this port caps 36h11 at 35 tags to fit ESP32 RAM, so your printed
  // tag IDs must be 0..34. IDs above that will never decode.
  apriltag_detector_add_family_bits(td, tf, 1);  // 1 bit, not the default 2
  // The port's README suggests nthreads=2 (one per core) is fastest; try it
  // after you have single-threaded detection working end to end.
  td->quad_decimate     = QUAD_DECIMATE;
  td->quad_sigma        = 0.0f;
  td->nthreads          = 1;
  td->refine_edges      = 1;
  td->decode_sharpening = 0.25;
  td->debug             = 0;
}

// Area of a perspective-skewed quad, from its two diagonals.
// Templated because this port converts the upstream doubles to float --
// this compiles either way, so you never have to chase a type mismatch here.
static inline float quadArea(const float p[4][2]) {
  float d1x = p[2][0] - p[0][0], d1y = p[2][1] - p[0][1];
  float d2x = p[3][0] - p[1][0], d2y = p[3][1] - p[1][1];
  return 0.5f * fabsf(d1x * d2y - d1y * d2x);
}

// -------------------------------------------------------------------------
static void visionTask(void *) {
  uint32_t nFrames = 0, nHits = 0;
  float    sumSide = 0, sumDm = 0, sumMs = 0;

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    image_u8_t im = {
      .width  = (int32_t)fb->width,
      .height = (int32_t)fb->height,
      .stride = (int32_t)fb->width,
      .buf    = fb->buf
    };

    uint32_t t0 = millis();
    zarray_t *dets = apriltag_detector_detect(td, &im);
    uint32_t dt = millis() - t0;

    apriltag_detection_t *best = nullptr;
    float bestArea = 0.0f;
    for (int i = 0; i < zarray_size(dets); i++) {
      apriltag_detection_t *det;
      zarray_get(dets, i, &det);
      if (det->decision_margin < MIN_DECISION_MARGIN) continue;
      float a = quadArea(det->p);
      if (a > bestArea) { bestArea = a; best = det; }
    }

    float side = best ? sqrtf(bestArea) : 0.0f;

    if (best) {
#if ENABLE_POSE
      float sx = (float)fb->width  / CAL_WIDTH;
      float sy = (float)fb->height / CAL_HEIGHT;
      apriltag_detection_info_t info;
      info.det     = best;
      info.tagsize = TAG_SIZE_M;
      info.fx = CAL_FX * sx;  info.fy = CAL_FY * sy;
      info.cx = CAL_CX * sx;  info.cy = CAL_CY * sy;
      apriltag_pose_t pose;
      estimate_tag_pose(&info, &pose);
      float z = (float)MATD_EL(pose.t, 2, 0);
      matd_destroy(pose.R); matd_destroy(pose.t);
      VISION_UART.printf("TAG,%d,%.1f,%.1f,%.1f,%.1f,%.3f\n",
                         best->id, best->c[0], best->c[1], side,
                         best->decision_margin, z);
#else
      VISION_UART.printf("TAG,%d,%.1f,%.1f,%.1f,%.1f\n",
                         best->id, best->c[0], best->c[1], side,
                         best->decision_margin);
#endif
    } else {
      VISION_UART.print("TAG,-1\n");
    }

    // ---- bench-test summary, one line per REPORT_EVERY frames ----
    nFrames++;
    sumMs += dt;
    if (best) { nHits++; sumSide += side; sumDm += best->decision_margin; }
    if (nFrames >= REPORT_EVERY) {
      Serial.printf("hit=%lu/%d  id=%d  side=%.1fpx  dm=%.0f  %.0fms\n",
                    (unsigned long)nHits, REPORT_EVERY,
                    best ? best->id : -1,
                    nHits ? sumSide / nHits : 0.0f,
                    nHits ? sumDm   / nHits : 0.0f,
                    sumMs / (float)REPORT_EVERY);
      nFrames = nHits = 0; sumSide = sumDm = sumMs = 0;
    }

    apriltag_detections_destroy(dets);
    esp_camera_fb_return(fb);
  }
}

// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  VISION_UART.begin(VISION_BAUD, SERIAL_8N1, VISION_RX_PIN, VISION_TX_PIN);

  if (!psramFound()) {
    Serial.println("FATAL: PSRAM not found. Tools > PSRAM > OPI PSRAM.");
    while (1) delay(1000);
  }
  Serial.printf("PSRAM free before init: %u\n", ESP.getFreePsram());

  if (!cameraInit()) {
    Serial.println("FATAL: camera init failed");
    while (1) delay(1000);
  }

  detectorInit();
  Serial.printf("PSRAM free after init:  %u\n", ESP.getFreePsram());
  Serial.printf("Internal heap free:     %u\n",
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  // Detection needs a deep stack; the default loop() stack will fault.
  xTaskCreatePinnedToCore(visionTask, "vision", 32768, nullptr, 5, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
