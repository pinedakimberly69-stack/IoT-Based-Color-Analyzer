// ===========================================================
// IoT Personal Color Analyzer
// ESP32 + TCS34725
// ===========================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <Preferences.h>

// ===========================================================
// SERVER CONFIGURATION
// ===========================================================
const uint16_t HTTP_PORT = 80;
WebServer server(HTTP_PORT);

// ===========================================================
// SENSOR CONFIGURATION
// ===========================================================
Adafruit_TCS34725 tcs(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_16X);

// ===========================================================
// WIFI MANAGER
// ===========================================================
const char *AP_SSID = "ColorAnalyzer";
const char *AP_PASSWORD = "12345678";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

bool wifiConnected = false;
Preferences preferences;

// ===========================================================
// VERSION INFORMATION
// ===========================================================
const String firmwareVersion = "8.2";
const String deviceName = "ESP32 Personal Color Analyzer";

// ===========================================================
// SCAN PARAMETERS
// ===========================================================
const unsigned long SENSOR_INTERVAL = 2000;
const int SAMPLE_COUNT = 12;
const int WARMUP_SAMPLES = 2;


const uint16_t MIN_CLEAR_COUNT = 250;

// Prevent near-saturation measurements from being accepted.
const uint16_t MAX_CLEAR_COUNT = 60000;
const uint16_t MAX_RGB_COUNT = 60000;


const float MIN_RECOMMENDED_LUX = 80.0f;
const float GOOD_MIN_LUX = 150.0f;
const float GOOD_MAX_LUX = 3000.0f;
const float MAX_RECOMMENDED_LUX = 10000.0f;

// Stability threshold: lower CV = more stable.
const float MAX_ACCEPTABLE_CV = 5.0f;
const float EXCELLENT_CV = 1.5f;

// ===========================================================
// PROTOTYPE COLOR CALIBRATION
// ===========================================================
// Start at 1.0. These should later be obtained from a controlled
// calibration procedure using a known neutral/white reference.
const float CAL_R = 1.000f;
const float CAL_G = 1.000f;
const float CAL_B = 1.000f;

// White-reference calibration. A matte neutral-white target should be
// measured at the exact sensor position and under the same illumination
// used for face scans. The values are stored in ESP32 NVS using Preferences.
float whiteRefR = 0.0f;
float whiteRefG = 0.0f;
float whiteRefB = 0.0f;
float whiteRefC = 0.0f;
bool whiteReferenceValid = false;

const char *PREF_NAMESPACE = "pca_cal";
const char *PREF_KEY_R = "whiteR";
const char *PREF_KEY_G = "whiteG";
const char *PREF_KEY_B = "whiteB";
const char *PREF_KEY_C = "whiteC";

const float MIN_WHITE_CLEAR = 500.0f;
const float MAX_WHITE_CLEAR = 60000.0f;


const float VERY_LIGHT_REFLECTANCE = 0.60f;
const float LIGHT_REFLECTANCE = 0.45f;
const float MEDIUM_REFLECTANCE = 0.30f;
const float SOFT_REFLECTANCE = 0.20f;

// ===========================================================
// DATA STRUCTURES
// ===========================================================
struct HSVColor {
  float hue;
  float saturation;
  float value;
};

struct LabColor {
  float L;
  float a;
  float b;
};

struct ColorReading {
  // Chromatic normalized RGB. These are ratios, NOT physical brightness.
  float redNorm;
  float greenNorm;
  float blueNorm;

  // Sensor values
  float lux;
  float colorTemperature;
  uint16_t rawR;
  uint16_t rawG;
  uint16_t rawB;
  uint16_t rawC;

  // White-reference calibrated reflectance (0..1, approximately).
  float reflectanceR;
  float reflectanceG;
  float reflectanceB;
  float reflectanceC;
  float brightnessIndex;

  // Skin features
  float hue;
  float saturation;
  float value;

  // CIE L*a*b*
  float labL;
  float laba;
  float labb;

  // Distances to undertone reference clusters
  float warmDistance;
  float coolDistance;
  float neutralDistance;

  float rbRatio;
  float rgRatio;
  float gbRatio;

  // Confidence and stability
  float confidence;
  float stability;
  float colorMatch;
  float undertoneClarity;
  float lightingQuality;

  // Scan quality
  bool acceptable;
  String scanQuality;
  String scanMessage;

  // Scores
  float warmScore;
  float coolScore;
  float neutralScore;
  float warmCoolIndex;

  // Results
  String hex;
  String undertone;
  String brightness;
  String contrast;
  String season;
};

ColorReading currentReading;
unsigned long lastSensorRead = 0;

// ===========================================================
// FUNCTION DECLARATIONS
// ===========================================================
HSVColor rgbToHSV(float r, float g, float b);
LabColor rgbToLab(float r, float g, float b);
float deltaE76(const LabColor &x, const LabColor &y);
float clampFloat(float value, float minimum, float maximum);
uint8_t ratioToByte(float ratio);
String rgbToHex(uint8_t r, uint8_t g, uint8_t b);
void calculateSkinFeatures(ColorReading &reading);
void calculateUndertoneScores(ColorReading &reading);
String determineUndertone(ColorReading &reading);
String detectBrightness(ColorReading &reading);
String detectContrast(ColorReading &reading);
String detectSeason(ColorReading &reading);
float calculateConfidence(ColorReading &reading);
void classifyColor(ColorReading &reading);
void readSensor(ColorReading &reading);
String createJSON(const ColorReading &reading);
String createStatusJSON();
void printReading(const ColorReading &reading);

void loadWhiteReference();
void saveWhiteReference();
void clearWhiteReference();
bool captureWhiteReference();
void printWhiteReference();
void handleSerialCommand();
void printHelp();

void resetReadingForScan(ColorReading &reading) {
  reading.hex = "#808080";
  reading.undertone = "-";
  reading.brightness = "-";
  reading.contrast = "-";
  reading.season = "-";
  reading.confidence = 0.0f;
  reading.stability = 0.0f;
  reading.colorMatch = 0.0f;
  reading.undertoneClarity = 0.0f;
  reading.lightingQuality = 0.0f;
  reading.warmDistance = 0.0f;
  reading.coolDistance = 0.0f;
  reading.neutralDistance = 0.0f;
  reading.warmScore = 0.0f;
  reading.coolScore = 0.0f;
  reading.neutralScore = 0.0f;
  reading.warmCoolIndex = 0.0f;
  reading.labL = 0.0f;
  reading.laba = 0.0f;
  reading.labb = 0.0f;
  reading.rawR = 0;
  reading.rawG = 0;
  reading.rawB = 0;
  reading.rawC = 0;
  reading.reflectanceR = 0.0f;
  reading.reflectanceG = 0.0f;
  reading.reflectanceB = 0.0f;
  reading.reflectanceC = 0.0f;
  reading.brightnessIndex = 0.0f;
}

// ===========================================================
// HELPERS
// ===========================================================
float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float safeRatio(float numerator, float denominator) {
  if (denominator <= 0.000001f) return 0.0f;
  return numerator / denominator;
}

uint8_t ratioToByte(float ratio) {
  // UI-only color representation. It represents chromaticity and should
  // not be interpreted as a calibrated camera RGB value.
  ratio = clampFloat(ratio, 0.0f, 1.0f);
  return (uint8_t)roundf(ratio * 255.0f);
}

String rgbToHex(uint8_t r, uint8_t g, uint8_t b) {
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "#%02X%02X%02X", r, g, b);
  return String(buffer);
}

// ===========================================================
// RGB -> HSV
// ===========================================================
HSVColor rgbToHSV(float r, float g, float b) {
  HSVColor out;

  float maximum = max(r, max(g, b));
  float minimum = min(r, min(g, b));
  float delta = maximum - minimum;

  out.value = maximum;
  out.saturation = (maximum <= 0.000001f) ? 0.0f : delta / maximum;
  out.hue = 0.0f;

  if (delta <= 0.000001f) {
    out.hue = 0.0f;
  } else if (maximum == r) {
    out.hue = 60.0f * fmodf(((g - b) / delta), 6.0f);
  } else if (maximum == g) {
    out.hue = 60.0f * (((b - r) / delta) + 2.0f);
  } else {
    out.hue = 60.0f * (((r - g) / delta) + 4.0f);
  }

  if (out.hue < 0.0f) out.hue += 360.0f;
  return out;
}

// ===========================================================
// sRGB -> CIELAB
// ===========================================================

LabColor rgbToLab(float r, float g, float b) {
  r = clampFloat(r, 0.0f, 1.0f);
  g = clampFloat(g, 0.0f, 1.0f);
  b = clampFloat(b, 0.0f, 1.0f);

  auto linearize = [](float c) -> float {
    if (c <= 0.04045f) return c / 12.92f;
    return powf((c + 0.055f) / 1.055f, 2.4f);
  };

  float R = linearize(r);
  float G = linearize(g);
  float B = linearize(b);

  // sRGB D65 -> XYZ
  float X = (0.4124564f * R) + (0.3575761f * G) + (0.1804375f * B);
  float Y = (0.2126729f * R) + (0.7151522f * G) + (0.0721750f * B);
  float Z = (0.0193339f * R) + (0.1191920f * G) + (0.9503041f * B);

  // D65 reference white
  const float Xn = 0.95047f;
  const float Yn = 1.00000f;
  const float Zn = 1.08883f;

  auto f = [](float t) -> float {
    const float delta = 6.0f / 29.0f;
    const float delta3 = delta * delta * delta;
    if (t > delta3) return cbrtf(t);
    return (t / (3.0f * delta * delta)) + (4.0f / 29.0f);
  };

  float fx = f(X / Xn);
  float fy = f(Y / Yn);
  float fz = f(Z / Zn);

  LabColor lab;
  lab.L = (116.0f * fy) - 16.0f;
  lab.a = 500.0f * (fx - fy);
  lab.b = 200.0f * (fy - fz);
  return lab;
}

float deltaE76(const LabColor &x, const LabColor &y) {
  float dL = x.L - y.L;
  float da = x.a - y.a;
  float db = x.b - y.b;
  return sqrtf((dL * dL) + (da * da) + (db * db));
}

// ===========================================================
// SKIN FEATURES
// ===========================================================
void calculateSkinFeatures(ColorReading &reading) {
  HSVColor hsv = rgbToHSV(
      reading.redNorm,
      reading.greenNorm,
      reading.blueNorm);

  reading.hue = hsv.hue;
  reading.saturation = hsv.saturation;
  reading.value = hsv.value;

  // Ratios are used only as chromatic descriptors.
  reading.rbRatio = safeRatio(reading.redNorm, max(reading.blueNorm, 0.001f));
  reading.rgRatio = safeRatio(reading.redNorm, max(reading.greenNorm, 0.001f));
  reading.gbRatio = safeRatio(reading.greenNorm, max(reading.blueNorm, 0.001f));
}

// ===========================================================
// UNDERTONE MATCHING
// ===========================================================

void calculateUndertoneScores(ColorReading &reading) {

  const float COOL_A = 10.0f;
  const float COOL_B = 8.0f;
  const float WARM_A = 16.0f;
  const float WARM_B = 24.0f;
  const float NEUTRAL_A = 13.0f;
  const float NEUTRAL_B = 16.0f;

  float va = WARM_A - COOL_A;
  float vb = WARM_B - COOL_B;
  float axisLengthSquared = (va * va) + (vb * vb);

  float pa = reading.laba - COOL_A;
  float pb = reading.labb - COOL_B;

  float projection = ((pa * va) + (pb * vb)) / axisLengthSquared;
  projection = clampFloat(projection, -0.25f, 1.25f);


  reading.warmCoolIndex = clampFloat(projection * 100.0f, 0.0f, 100.0f);

  // Distances are retained for diagnostics and API compatibility.
  float daWarm = reading.laba - WARM_A;
  float dbWarm = reading.labb - WARM_B;
  float daCool = reading.laba - COOL_A;
  float dbCool = reading.labb - COOL_B;
  float daNeutral = reading.laba - NEUTRAL_A;
  float dbNeutral = reading.labb - NEUTRAL_B;

  reading.warmDistance = sqrtf((daWarm * daWarm) + (dbWarm * dbWarm));
  reading.coolDistance = sqrtf((daCool * daCool) + (dbCool * dbCool));
  reading.neutralDistance = sqrtf((daNeutral * daNeutral) + (dbNeutral * dbNeutral));

  // Scores are based on the measured position along the Cool-Warm axis.
  // Neutral peaks around the middle and falls away smoothly toward either end.
  float warmPosition = reading.warmCoolIndex;
  float coolPosition = 100.0f - warmPosition;
  float neutralPosition = 100.0f - (fabsf(warmPosition - 50.0f) * 2.0f);

  reading.warmScore = clampFloat(warmPosition, 0.0f, 100.0f);
  reading.coolScore = clampFloat(coolPosition, 0.0f, 100.0f);
  reading.neutralScore = clampFloat(neutralPosition, 0.0f, 100.0f);
}

String determineUndertone(ColorReading &reading) {
  calculateUndertoneScores(reading);

  // A neutral band avoids forcing a Warm/Cool result when the measured skin
  // color sits close to the midpoint of the reference axis.
  const float COOL_LIMIT = 40.0f;
  const float WARM_LIMIT = 60.0f;

  float position = reading.warmCoolIndex;

  if (position < COOL_LIMIT) {
    reading.undertoneClarity =
        clampFloat(((COOL_LIMIT - position) / COOL_LIMIT) * 100.0f,
                   0.0f, 100.0f);
    return "Cool";
  }

  if (position > WARM_LIMIT) {
    reading.undertoneClarity =
        clampFloat(((position - WARM_LIMIT) / (100.0f - WARM_LIMIT)) * 100.0f,
                   0.0f, 100.0f);
    return "Warm";
  }

  // Inside the neutral band, clarity is intentionally low because the sensor
  // does not have enough chromatic separation to justify Warm/Cool.
  float distanceFromCenter = fabsf(position - 50.0f);
  reading.undertoneClarity =
      clampFloat((10.0f - distanceFromCenter) * 10.0f, 0.0f, 100.0f);
  return "Neutral";
}

// ===========================================================
// BRIGHTNESS
// ===========================================================

String detectBrightness(ColorReading &reading) {

  if (!whiteReferenceValid || whiteRefC <= 0.0f) {
    return "Uncalibrated";
  }

  float relative = reading.reflectanceC;
  if (relative >= VERY_LIGHT_REFLECTANCE) return "Very Light";
  if (relative >= LIGHT_REFLECTANCE) return "Light";
  if (relative >= MEDIUM_REFLECTANCE) return "Medium";
  if (relative >= SOFT_REFLECTANCE) return "Soft";
  return "Deep";
}

// ===========================================================
// CONTRAST
// ===========================================================
String detectContrast(ColorReading &reading) {
  float maximum = max(
      reading.redNorm,
      max(reading.greenNorm, reading.blueNorm));
  float minimum = min(
      reading.redNorm,
      min(reading.greenNorm, reading.blueNorm));

  float difference = maximum - minimum;

  if (difference < 0.12f) return "Low";
  if (difference < 0.22f) return "Medium";
  return "High";
}

// ===========================================================
// SEASON CLASSIFIER
// ===========================================================

String detectSeason(ColorReading &reading) {
  int spring = 0;
  int summer = 0;
  int autumn = 0;
  int winter = 0;

  if (reading.undertone == "Warm") {
    spring += 4;
    autumn += 4;
  } else if (reading.undertone == "Cool") {
    summer += 4;
    winter += 4;
  } else {
    spring += 2;
    summer += 2;
    autumn += 2;
    winter += 2;
  }

  if (reading.brightness == "Very Light") {
    spring += 3;
    summer += 3;
  } else if (reading.brightness == "Light") {
    spring += 2;
    summer += 2;
  } else if (reading.brightness == "Medium") {
    spring += 1;
    summer += 1;
    autumn += 2;
  } else if (reading.brightness == "Soft") {
    summer += 2;
    autumn += 2;
  } else {
    autumn += 3;
    winter += 3;
  }

  if (reading.contrast == "High") {
    winter += 3;
    spring += 2;
  } else if (reading.contrast == "Medium") {
    autumn += 2;
    summer += 2;
  } else {
    summer += 3;
    autumn += 3;
  }

  if (reading.saturation >= 0.65f) {
    spring += 2;
    winter += 2;
  } else if (reading.saturation >= 0.40f) {
    autumn += 2;
  } else {
    summer += 2;
  }

  String season = "Spring";
  int highest = spring;

  if (summer > highest) {
    highest = summer;
    season = "Summer";
  }
  if (autumn > highest) {
    highest = autumn;
    season = "Autumn";
  }
  if (winter > highest) {
    highest = winter;
    season = "Winter";
  }

  if (season == "Spring") {
    if (reading.brightness == "Very Light") return "Light Spring";
    if (reading.contrast == "High") return "Bright Spring";
    return "True Spring";
  }

  if (season == "Summer") {
    if (reading.brightness == "Very Light") return "Light Summer";
    if (reading.saturation < 0.40f) return "Soft Summer";
    return "True Summer";
  }

  if (season == "Autumn") {
    if (reading.brightness == "Deep") return "Deep Autumn";
    if (reading.saturation < 0.40f) return "Soft Autumn";
    return "True Autumn";
  }

  if (reading.brightness == "Deep") return "Deep Winter";
  if (reading.contrast == "High") return "Bright Winter";
  return "True Winter";
}

// ===========================================================
// LIGHTING QUALITY
// ===========================================================
float calculateLightingQuality(float lux, uint16_t clearCount) {

  (void)lux;

  if (clearCount < MIN_CLEAR_COUNT) {
    return clampFloat(((float)clearCount / MIN_CLEAR_COUNT) * 50.0f, 0.0f, 50.0f);
  }

  if (clearCount < 1000) {
    return 70.0f + ((float)(clearCount - MIN_CLEAR_COUNT) /
                    (1000.0f - MIN_CLEAR_COUNT)) * 30.0f;
  }

  if (clearCount <= 20000) return 100.0f;

  if (clearCount <= 50000) {
    return 100.0f - ((float)(clearCount - 20000) / 30000.0f) * 50.0f;
  }

  return 0.0f;
}

// ===========================================================
// CONFIDENCE
// ===========================================================
float calculateConfidence(ColorReading &reading) {
  float stabilityScore = clampFloat(reading.stability, 0.0f, 100.0f);
  float colorMatch = max(reading.warmScore,
                         max(reading.coolScore, reading.neutralScore));

  // White-reference calibration is required for a final result.
  float calibrationScore = whiteReferenceValid ? 100.0f : 55.0f;

  float confidence =
      (stabilityScore * 0.35f) +
      (reading.lightingQuality * 0.20f) +
      (colorMatch * 0.20f) +
      (reading.undertoneClarity * 0.15f) +
      (calibrationScore * 0.10f);

  if (reading.rawC < MIN_CLEAR_COUNT) confidence *= 0.45f;
  // Do not penalize based on calculated lux: the onboard LED is the controlled
  // light source and lux changes with the target's reflectance.
  if (!whiteReferenceValid) confidence = min(confidence, 74.0f);

  reading.colorMatch = colorMatch;
  return clampFloat(confidence, 0.0f, 100.0f);
}

// ===========================================================
// COLOR CLASSIFICATION
// ===========================================================
void classifyColor(ColorReading &reading) {
  calculateSkinFeatures(reading);
  reading.undertone = determineUndertone(reading);
  reading.brightness = detectBrightness(reading);
  reading.contrast = detectContrast(reading);
  reading.season = detectSeason(reading);
  reading.lightingQuality = calculateLightingQuality(
      reading.lux,
      reading.rawC);
  reading.confidence = calculateConfidence(reading);

  // Quality status is intentionally stricter than the old firmware.
  if (!whiteReferenceValid) {
    reading.acceptable = false;
    reading.scanQuality = "CALIBRATION_REQUIRED";
    reading.scanMessage =
        "White reference is not calibrated. Capture a matte white reference first.";
  } else if (reading.rawC < MIN_CLEAR_COUNT) {
    reading.acceptable = false;
    reading.scanQuality = "RESCAN_REQUIRED";
    reading.scanMessage =
        "Sensor signal is too low. Keep the onboard LED and sensor position consistent, then rescan.";
  } else if (reading.rawC > MAX_CLEAR_COUNT ||
             reading.rawR > MAX_RGB_COUNT ||
             reading.rawG > MAX_RGB_COUNT ||
             reading.rawB > MAX_RGB_COUNT) {
    reading.acceptable = false;
    reading.scanQuality = "RESCAN_REQUIRED";
    reading.scanMessage =
        "Sensor signal is too strong or near saturation. Check the sensor setup and rescan.";
  } else if (reading.confidence >= 80.0f) {
    reading.acceptable = true;
    reading.scanQuality = "ACCEPTABLE";
    reading.scanMessage =
        "Good lighting and stable readings were obtained.";
  } else if (reading.confidence >= 60.0f) {
    reading.acceptable = false;
    reading.scanQuality = "RESCAN_RECOMMENDED";
    reading.scanMessage =
        "The reading is moderately reliable. Rescan for greater confidence.";
  } else {
    reading.acceptable = false;
    reading.scanQuality = "RESCAN_REQUIRED";
    reading.scanMessage =
        "The reading is not reliable enough. Keep the sensor steady and rescan.";
  }
}

// ===========================================================
// SENSOR READING
// ===========================================================
void readSensor(ColorReading &reading) {
  resetReadingForScan(reading);
  reading.acceptable = false;
  reading.scanQuality = "SCANNING";
  reading.scanMessage = "Collecting sensor readings.";

  // Warm-up reads allow the sensor to settle after placement.
  for (int i = 0; i < WARMUP_SAMPLES; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    delay(60);
  }

  uint32_t totalR = 0;
  uint32_t totalG = 0;
  uint32_t totalB = 0;
  uint32_t totalC = 0;

  double totalR2 = 0.0;
  double totalG2 = 0.0;
  double totalB2 = 0.0;
  double totalC2 = 0.0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    uint16_t r, g, b, c;

    tcs.getRawData(&r, &g, &b, &c);

    totalR += r;
    totalG += g;
    totalB += b;
    totalC += c;

    totalR2 += (double)r * (double)r;
    totalG2 += (double)g * (double)g;
    totalB2 += (double)b * (double)b;
    totalC2 += (double)c * (double)c;

    delay(60);
  }

  uint16_t avgR = totalR / SAMPLE_COUNT;
  uint16_t avgG = totalG / SAMPLE_COUNT;
  uint16_t avgB = totalB / SAMPLE_COUNT;
  uint16_t avgC = totalC / SAMPLE_COUNT;

  auto channelCV = [](double sum, double sumSquares) -> float {
    double mean = sum / (double)SAMPLE_COUNT;
    if (mean <= 0.0) return 100.0f;

    double variance =
        (sumSquares / (double)SAMPLE_COUNT) - (mean * mean);

    if (variance < 0.0) variance = 0.0;

    double standardDeviation = sqrt(variance);
    return (float)((standardDeviation / mean) * 100.0);
  };

  float cvR = channelCV(totalR, totalR2);
  float cvG = channelCV(totalG, totalG2);
  float cvB = channelCV(totalB, totalB2);
  float cvC = channelCV(totalC, totalC2);
  float averageCV = (cvR + cvG + cvB + cvC) / 4.0f;

  // stability = 100 at 0% CV and 0 at/above 5% CV.
  reading.stability = clampFloat(
      100.0f - ((averageCV / MAX_ACCEPTABLE_CV) * 100.0f),
      0.0f,
      100.0f);

  reading.rawR = avgR;
  reading.rawG = avgG;
  reading.rawB = avgB;
  reading.rawC = avgC;

  // Reject clearly invalid / near-saturated readings.
  if (avgC < MIN_CLEAR_COUNT) {
    reading.lux = tcs.calculateLux(avgR, avgG, avgB);
    if (reading.lux < 0) reading.lux = 0;
    reading.colorTemperature = tcs.calculateColorTemperature(avgR, avgG, avgB);
    reading.confidence = 0.0f;
    reading.scanQuality = "RESCAN_REQUIRED";
    reading.scanMessage =
        "Sensor signal is too low. Increase the controlled light and rescan.";
    reading.acceptable = false;

    Serial.println("Rejected: Low sensor signal");
    Serial.print("Average C : ");
    Serial.println(avgC);
    return;
  }

  if (avgC > MAX_CLEAR_COUNT ||
      avgR > MAX_RGB_COUNT ||
      avgG > MAX_RGB_COUNT ||
      avgB > MAX_RGB_COUNT) {
    reading.confidence = 0.0f;
    reading.scanQuality = "RESCAN_REQUIRED";
    reading.scanMessage =
        "Sensor signal is too strong. Reduce the light and rescan.";
    reading.acceptable = false;

    Serial.println("Rejected: Sensor signal too high");
    return;
  }

  // ----------------------------------------------------------
  // White-reference calibrated RGB
  // ----------------------------------------------------------
  float correctedR = avgR * CAL_R;
  float correctedG = avgG * CAL_G;
  float correctedB = avgB * CAL_B;

  if (whiteReferenceValid) {
    reading.reflectanceR = clampFloat(correctedR / whiteRefR, 0.0f, 1.25f);
    reading.reflectanceG = clampFloat(correctedG / whiteRefG, 0.0f, 1.25f);
    reading.reflectanceB = clampFloat(correctedB / whiteRefB, 0.0f, 1.25f);
    reading.reflectanceC = clampFloat(((float)avgC) / whiteRefC, 0.0f, 1.25f);
    reading.brightnessIndex = reading.reflectanceC * 100.0f;
  } else {
    // Before calibration, retain chromatic-only behavior for diagnostics.
    reading.reflectanceR = 0.0f;
    reading.reflectanceG = 0.0f;
    reading.reflectanceB = 0.0f;
    reading.reflectanceC = 0.0f;
    reading.brightnessIndex = 0.0f;
  }

  float displayR = whiteReferenceValid ? reading.reflectanceR : correctedR;
  float displayG = whiteReferenceValid ? reading.reflectanceG : correctedG;
  float displayB = whiteReferenceValid ? reading.reflectanceB : correctedB;

  // Convert calibrated reflectance into an sRGB-like display value.
  reading.hex = rgbToHex(
      ratioToByte(clampFloat(displayR, 0.0f, 1.0f)),
      ratioToByte(clampFloat(displayG, 0.0f, 1.0f)),
      ratioToByte(clampFloat(displayB, 0.0f, 1.0f)));

  float rgbTotal = correctedR + correctedG + correctedB;
  if (rgbTotal <= 0.0f) rgbTotal = 1.0f;

  reading.redNorm = correctedR / rgbTotal;
  reading.greenNorm = correctedG / rgbTotal;
  reading.blueNorm = correctedB / rgbTotal;

  // For Lab/undertone, use white-reference calibrated RGB when available.
  if (whiteReferenceValid) {
    float labR = clampFloat(reading.reflectanceR, 0.0f, 1.0f);
    float labG = clampFloat(reading.reflectanceG, 0.0f, 1.0f);
    float labB = clampFloat(reading.reflectanceB, 0.0f, 1.0f);
    LabColor calibratedLab = rgbToLab(labR, labG, labB);
    reading.labL = calibratedLab.L;
    reading.laba = calibratedLab.a;
    reading.labb = calibratedLab.b;
  }

  // ----------------------------------------------------------
  // Lux & color temperature
  // ----------------------------------------------------------
  reading.lux = tcs.calculateLux(avgR, avgG, avgB);
  if (reading.lux < 0) reading.lux = 0;
  if (reading.lux > 10000) reading.lux = 10000;

  reading.colorTemperature =
      tcs.calculateColorTemperature(avgR, avgG, avgB);

  if (reading.colorTemperature < 1500) reading.colorTemperature = 1500;
  if (reading.colorTemperature > 12000) reading.colorTemperature = 12000;

  classifyColor(reading);

  printReading(reading);
}

// ===========================================================
// WHITE-REFERENCE CALIBRATION
// ===========================================================
void loadWhiteReference() {
  preferences.begin(PREF_NAMESPACE, true);
  whiteRefR = preferences.getFloat(PREF_KEY_R, 0.0f);
  whiteRefG = preferences.getFloat(PREF_KEY_G, 0.0f);
  whiteRefB = preferences.getFloat(PREF_KEY_B, 0.0f);
  whiteRefC = preferences.getFloat(PREF_KEY_C, 0.0f);
  preferences.end();

  whiteReferenceValid =
      whiteRefR >= MIN_WHITE_CLEAR &&
      whiteRefG >= MIN_WHITE_CLEAR &&
      whiteRefB >= MIN_WHITE_CLEAR &&
      whiteRefC >= MIN_WHITE_CLEAR &&
      whiteRefR <= MAX_WHITE_CLEAR &&
      whiteRefG <= MAX_WHITE_CLEAR &&
      whiteRefB <= MAX_WHITE_CLEAR &&
      whiteRefC <= MAX_WHITE_CLEAR;
}

void saveWhiteReference() {
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putFloat(PREF_KEY_R, whiteRefR);
  preferences.putFloat(PREF_KEY_G, whiteRefG);
  preferences.putFloat(PREF_KEY_B, whiteRefB);
  preferences.putFloat(PREF_KEY_C, whiteRefC);
  preferences.end();
  whiteReferenceValid = true;
}

void clearWhiteReference() {
  preferences.begin(PREF_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  whiteRefR = whiteRefG = whiteRefB = whiteRefC = 0.0f;
  whiteReferenceValid = false;
  Serial.println("White reference cleared.");
}

void printWhiteReference() {
  Serial.println();
  Serial.println("----------- WHITE REFERENCE -----------");
  if (!whiteReferenceValid) {
    Serial.println("Status : NOT CALIBRATED");
  } else {
    Serial.println("Status : CALIBRATED");
    Serial.print("White R : "); Serial.println(whiteRefR, 2);
    Serial.print("White G : "); Serial.println(whiteRefG, 2);
    Serial.print("White B : "); Serial.println(whiteRefB, 2);
    Serial.print("White C : "); Serial.println(whiteRefC, 2);
  }
  Serial.println("---------------------------------------");
}

bool captureWhiteReference() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("WHITE REFERENCE CALIBRATION");
  Serial.println("Keep the matte white reference at the exact");
  Serial.println("same distance/position used for face scans.");
  Serial.println("Collecting samples...");

  uint32_t totalR = 0, totalG = 0, totalB = 0, totalC = 0;
  const int calibrationSamples = 20;

  for (int i = 0; i < 4; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    delay(60);
  }

  for (int i = 0; i < calibrationSamples; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    totalR += r;
    totalG += g;
    totalB += b;
    totalC += c;
    delay(60);
  }

  whiteRefR = (float)totalR / calibrationSamples;
  whiteRefG = (float)totalG / calibrationSamples;
  whiteRefB = (float)totalB / calibrationSamples;
  whiteRefC = (float)totalC / calibrationSamples;

  float whiteLux = tcs.calculateLux(
      (uint16_t)whiteRefR, (uint16_t)whiteRefG, (uint16_t)whiteRefB);
  if (whiteLux < 0.0f) whiteLux = 0.0f;
  Serial.print("White reference calculated lux (diagnostic only): ");
  Serial.println(whiteLux, 2);

  if (whiteRefR < MIN_WHITE_CLEAR || whiteRefG < MIN_WHITE_CLEAR ||
      whiteRefB < MIN_WHITE_CLEAR || whiteRefC < MIN_WHITE_CLEAR ||
      whiteRefR > MAX_WHITE_CLEAR || whiteRefG > MAX_WHITE_CLEAR ||
      whiteRefB > MAX_WHITE_CLEAR || whiteRefC > MAX_WHITE_CLEAR) {
    whiteReferenceValid = false;
    Serial.println("Calibration FAILED: white reference signal is outside the valid range.");
    printWhiteReference();
    return false;
  }

  saveWhiteReference();
  Serial.println("Calibration SUCCESS.");
  printWhiteReference();
  Serial.println("You can now replace the white reference with face skin and scan.");
  return true;
}

void printHelp() {
  Serial.println();
  Serial.println("=========== V8.2 SERIAL COMMANDS ===========");
  Serial.println("W = capture/store white reference");
  Serial.println("P = print white reference status");
  Serial.println("C = clear white reference");
  Serial.println("S = perform an immediate face scan");
  Serial.println("H = show this help");
  Serial.println("===========================================");
}

void handleSerialCommand() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\n' || command == '\r' || command == ' ') continue;

    if (command == 'W' || command == 'w') {
      captureWhiteReference();
    } else if (command == 'P' || command == 'p') {
      printWhiteReference();
    } else if (command == 'C' || command == 'c') {
      clearWhiteReference();
    } else if (command == 'S' || command == 's') {
      readSensor(currentReading);
    } else if (command == 'H' || command == 'h' || command == '?') {
      printHelp();
    }
  }
}

// ===========================================================
// JSON
// ===========================================================
String createJSON(const ColorReading &reading) {
  JsonDocument doc;

  doc["device"] = deviceName;
  doc["firmware"] = firmwareVersion;
  doc["hex"] = reading.hex;
  doc["undertone"] = reading.undertone;
  doc["brightness"] = reading.brightness;
  doc["contrast"] = reading.contrast;
  doc["season"] = reading.season;
  doc["confidence"] = reading.confidence;
  doc["stability"] = reading.stability;
  doc["acceptable"] = reading.acceptable;
  doc["scanQuality"] = reading.scanQuality;
  doc["scanMessage"] = reading.scanMessage;
  doc["lux"] = reading.lux;
  doc["colorTemp"] = reading.colorTemperature;

  doc["red"] = reading.redNorm;
  doc["green"] = reading.greenNorm;
  doc["blue"] = reading.blueNorm;

  doc["rawR"] = reading.rawR;
  doc["rawG"] = reading.rawG;
  doc["rawB"] = reading.rawB;
  doc["rawC"] = reading.rawC;
  doc["whiteReferenceCalibrated"] = whiteReferenceValid;
  doc["whiteR"] = whiteRefR;
  doc["whiteG"] = whiteRefG;
  doc["whiteB"] = whiteRefB;
  doc["whiteC"] = whiteRefC;
  doc["reflectanceR"] = reading.reflectanceR;
  doc["reflectanceG"] = reading.reflectanceG;
  doc["reflectanceB"] = reading.reflectanceB;
  doc["reflectanceC"] = reading.reflectanceC;
  doc["brightnessIndex"] = reading.brightnessIndex;

  doc["labL"] = reading.labL;
  doc["laba"] = reading.laba;
  doc["labb"] = reading.labb;

  doc["warmDistance"] = reading.warmDistance;
  doc["coolDistance"] = reading.coolDistance;
  doc["neutralDistance"] = reading.neutralDistance;

  doc["warmScore"] = reading.warmScore;
  doc["coolScore"] = reading.coolScore;
  doc["neutralScore"] = reading.neutralScore;
  doc["warmCoolIndex"] = reading.warmCoolIndex;
  doc["colorMatch"] = reading.colorMatch;
  doc["undertoneClarity"] = reading.undertoneClarity;
  doc["lightingQuality"] = reading.lightingQuality;

  String json;
  serializeJsonPretty(doc, json);
  return json;
}

String createStatusJSON() {
  JsonDocument doc;

  doc["device"] = deviceName;
  doc["firmware"] = firmwareVersion;
  doc["wifi"] = wifiConnected;
  doc["ssid"] = AP_SSID;
  doc["ip"] = WiFi.softAPIP().toString();
  // In standalone AP mode, the ESP32 has no router RSSI.
  // Keep the field for Flutter compatibility.
  doc["signal"] = 0;
  doc["clients"] = WiFi.softAPgetStationNum();
  doc["uptime"] = millis() / 1000;

  String json;
  serializeJsonPretty(doc, json);
  return json;
}

// ===========================================================
// SERIAL OUTPUT
// ===========================================================
void printReading(const ColorReading &reading) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("Raw R : "); Serial.println(reading.rawR);
  Serial.print("Raw G : "); Serial.println(reading.rawG);
  Serial.print("Raw B : "); Serial.println(reading.rawB);
  Serial.print("Raw C : "); Serial.println(reading.rawC);
  if (whiteReferenceValid) {
    Serial.print("Reflectance R : "); Serial.println(reading.reflectanceR, 3);
    Serial.print("Reflectance G : "); Serial.println(reading.reflectanceG, 3);
    Serial.print("Reflectance B : "); Serial.println(reading.reflectanceB, 3);
    Serial.print("Reflectance C : "); Serial.println(reading.reflectanceC, 3);
  }
  Serial.println();
  Serial.print("HEX : "); Serial.println(reading.hex);
  Serial.print("Undertone : "); Serial.println(reading.undertone);
  Serial.print("Brightness : "); Serial.println(reading.brightness);
  Serial.print("Contrast : "); Serial.println(reading.contrast);
  Serial.print("Season : "); Serial.println(reading.season);
  Serial.print("Lux : "); Serial.println(reading.lux, 2);
  Serial.print("Color Temp : "); Serial.println(reading.colorTemperature, 2);
  Serial.println();
  Serial.print("Lab L* : "); Serial.println(reading.labL, 2);
  Serial.print("Lab a* : "); Serial.println(reading.laba, 2);
  Serial.print("Lab b* : "); Serial.println(reading.labb, 2);
  Serial.println();
  Serial.print("Warm Distance : "); Serial.println(reading.warmDistance, 2);
  Serial.print("Cool Distance : "); Serial.println(reading.coolDistance, 2);
  Serial.print("Neutral Distance : "); Serial.println(reading.neutralDistance, 2);
  Serial.print("Warm Score : "); Serial.println(reading.warmScore, 2);
  Serial.print("Cool Score : "); Serial.println(reading.coolScore, 2);
  Serial.print("Neutral Score : "); Serial.println(reading.neutralScore, 2);
  Serial.print("Warm-Cool Index : "); Serial.println(reading.warmCoolIndex, 2);
  Serial.println();
  Serial.print("Stability : "); Serial.print(reading.stability, 2); Serial.println("%");
  Serial.print("Color Match : "); Serial.print(reading.colorMatch, 2); Serial.println("%");
  Serial.print("Undertone Clarity : "); Serial.print(reading.undertoneClarity, 2); Serial.println("%");
  Serial.print("Lighting Quality : "); Serial.print(reading.lightingQuality, 2); Serial.println("%");
  Serial.print("Confidence : "); Serial.print(reading.confidence, 2); Serial.println("%");
  Serial.print("Scan Quality : "); Serial.println(reading.scanQuality);
  Serial.print("Message : "); Serial.println(reading.scanMessage);
  Serial.println("========================================");
}

// ===========================================================
// WEB SERVER
// ===========================================================
void handleRoot() {
  String html;
  html += "<!DOCTYPE html><html><head>";
  html += "<title>ESP32 Personal Color Analyzer</title>";
  html += "<meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "</head><body style='font-family:Arial;text-align:center;padding:30px;'>";
  html += "<h2>IoT Personal Color Analyzer</h2>";
  html += "<p><b>Firmware:</b> " + firmwareVersion + "</p>";
  html += "<p><b>Device:</b> " + deviceName + "</p>";
  html += "<p><b>White Reference:</b> " + String(whiteReferenceValid ? "Calibrated" : "Not Calibrated") + "</p>";
  html += "<hr><h3>Available API</h3>";
  html += "<p><a href='/color'>/color</a></p>";
  html += "<p><a href='/status'>/status</a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleColor() {
  server.send(200, "application/json", createJSON(currentReading));
}

void handleStatus() {
  server.send(200, "application/json", createStatusJSON());
}

void handleNotFound() {
  JsonDocument doc;
  doc["status"] = 404;
  doc["message"] = "Endpoint not found";
  doc["available"] = "/color , /status";

  String json;
  serializeJsonPretty(doc, json);
  server.send(404, "application/json", json);
}

void startWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/color", HTTP_GET, handleColor);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println();
  Serial.println("=================================");
  Serial.println("HTTP Server Started");
  Serial.print("Home : http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Serial.print("Color API : http://");
  Serial.print(WiFi.localIP());
  Serial.println("/color");
  Serial.print("Status API : http://");
  Serial.print(WiFi.localIP());
  Serial.println("/status");
  Serial.println("=================================");
}

// ===========================================================
// WIFI
// ===========================================================
void connectWiFi() {
  Serial.println();
  Serial.println("Starting standalone ColorAnalyzer Wi-Fi...");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
    Serial.println("WARNING: Failed to configure AP IP.");
  }

  // ESP32 creates its own Wi-Fi network. No router or internet is needed.
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("ERROR: Failed to start ColorAnalyzer Wi-Fi.");
    wifiConnected = false;
    return;
  }

  wifiConnected = true;

  Serial.println();
  Serial.println("=================================");
  Serial.println("ColorAnalyzer Wi-Fi Started");
  Serial.print("SSID : "); Serial.println(AP_SSID);
  Serial.print("Password : "); Serial.println(AP_PASSWORD);
  Serial.print("IP Address : "); Serial.println(WiFi.softAPIP());
  Serial.println("Connect your phone directly to this Wi-Fi.");
  Serial.println("No router or internet connection is required.");
  Serial.println("=================================");
}

// ===========================================================
// SETUP
// ===========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("IoT Personal Color Analyzer - V8.2 Standalone AP");
  Serial.println("ESP32 + TCS34725");
  Serial.println("========================================");

  Wire.begin(21, 22);

  if (!tcs.begin()) {
    Serial.println("ERROR: TCS34725 NOT FOUND");
    while (true) delay(1000);
  }

  Serial.println("TCS34725 Initialized");
  Serial.println("Integration: 50 ms");
  Serial.println("Gain: 16X");

  loadWhiteReference();
  printWhiteReference();
  printHelp();

  connectWiFi();
  startWebServer();

  // First reading.
  readSensor(currentReading);
}

// ===========================================================
// LOOP
// ===========================================================
void loop() {
  handleSerialCommand();
  server.handleClient();

  // Standalone AP mode: no router/station reconnection is required.

  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    readSensor(currentReading);
  }
}
