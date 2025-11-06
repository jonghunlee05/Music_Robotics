#include <Wire.h>

// ---- I2C + device
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68   // your board responds here

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_TEMP_OUT_H   = 0x41;
static const uint8_t REG_GYRO_XOUT_H  = 0x43;
static const uint8_t REG_WHO_AM_I     = 0x75;

// ---- chosen ranges
#define ACCEL_FS_SEL 0   // ±2g
#define GYRO_FS_SEL  0   // ±250 dps

// sensitivity LSB per unit for chosen ranges
static const float ACCEL_LSB_PER_G = 16384.0f; // for ±2g
static const float GYRO_LSB_PER_DPS = 131.0f;  // for ±250 dps

// ---- tiny I2C helpers
uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0xFF;
}
void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}
void readN(uint8_t startReg, uint8_t len, uint8_t* buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len, (uint8_t)true);
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
}

// ---- setup serial robustly (ESP32 likes to reset when monitor opens)
void robustSerialBegin(unsigned long baud = 115200) {
  Serial.begin(baud);
  for (int i = 0; i < 20; i++) { delay(100); if (Serial) break; }
  delay(200);
  Serial.println();
  Serial.printf(">>> Serial up at %lu\n", baud);
}

// ---- complementary filter state
float pitch_deg = 0.0f, roll_deg = 0.0f;
unsigned long last_ms = 0;

// ---- hit detection state
// We detect a hit when |a| spikes noticeably above 1g (gravity).
// Very cheap high-pass by subtracting a slow EMA of |a|.
float a_mag_ema = 1.0f;         // tracks gravity baseline ~1g
const float A_EMA_ALPHA = 0.02f; // lower = slower baseline
const float HIT_THRESH_G = 0.35f; // spike above baseline (g units) that counts as a hit
const unsigned HIT_REFRACT_MS = 120; // minimum gap between hits
unsigned long last_hit_ms = 0;
float last_spike_g = 0.0f;

// Optional: require downward-ish swing to reduce false positives
// We check if az component dips below gravity meaning hand is accelerating down.
const float DOWN_G_EXTRA = -0.20f; // require az_g < (1.0 + DOWN_G_EXTRA) at impact

// Map spike height to 1..127 "velocity"
int spikeToMidi(float spike_g) {
  float v = spike_g;               // ~0.35 .. maybe 2.0 on big swings
  if (v < 0) v = 0;
  float scaled = v * 90.0f + 20.0f; // crude mapping
  if (scaled < 1) scaled = 1;
  if (scaled > 127) scaled = 127;
  return (int)scaled;
}

// Decide which drum based on orientation at hit moment
const char* mapZone(float roll, float pitch) {
  // Priority: if you tip the stick steeply down (pitch very negative), call it Kick
  if (pitch < -60.0f) return "Kick";

  if (roll < -20.0f)       return "Tom1";
  else if (roll > 20.0f)   return "HiHat";
  else                     return "Snare";
}

// ---- optional: print raw for debugging
const bool VERBOSE_STREAM = false;

void setup() {
  robustSerialBegin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // keep it chill for clones

  // Wake device
  write8(REG_PWR_MGMT_1, 0x00); // clear sleep
  delay(10);

  // Sanity check identity
  uint8_t who = read8(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X (expected 0x68/0x70 variants ok)\n", who);

  // Low-pass filter / sample rate config
  write8(REG_SMPLRT_DIV, 0x07);     // sample rate divider
  write8(REG_CONFIG,     0x03);     // DLPF ~44Hz (for 1kHz internal)
  write8(REG_GYRO_CONFIG,  (GYRO_FS_SEL  << 3));
  write8(REG_ACCEL_CONFIG, (ACCEL_FS_SEL << 3));
  delay(50);

  // Prime filter timestamp
  last_ms = millis();

  // Prime magnitude EMA with 1g
  a_mag_ema = 1.0f;
}

void loop() {
  // Read 14 bytes: Accel XYZ, Temp, Gyro XYZ
  uint8_t buf[14];
  readN(REG_ACCEL_XOUT_H, 14, buf);

  auto s16 = [&](int i)->int16_t { return (int16_t)((buf[i] << 8) | buf[i+1]); };

  int16_t ax_raw = s16(0), ay_raw = s16(2), az_raw = s16(4);
  int16_t t_raw  = s16(6);
  int16_t gx_raw = s16(8), gy_raw = s16(10), gz_raw = s16(12);

  // Convert to units
  float ax_g = ax_raw / ACCEL_LSB_PER_G;
  float ay_g = ay_raw / ACCEL_LSB_PER_G;
  float az_g = az_raw / ACCEL_LSB_PER_G;

  float gx_dps = gx_raw / GYRO_LSB_PER_DPS;
  float gy_dps = gy_raw / GYRO_LSB_PER_DPS;
  float gz_dps = gz_raw / GYRO_LSB_PER_DPS;

  // Temperature (MPU datasheet: 36.53°C + t_raw/340 for 6050/6500 family)
  float temp_c = 36.53f + (t_raw / 340.0f);

  // Complementary filter for roll/pitch (yaw needs mag or integration drift handling)
  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  // accel tilt estimate (degrees)
  float pitch_acc = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;
  float roll_acc  = atan2f( ay_g, az_g ) * 180.0f / PI;

  // integrate gyro
  pitch_deg += gy_dps * dt;   // gyro y affects pitch (depends on axis convention)
  roll_deg  += gx_dps * dt;   // gyro x affects roll

  // fuse with accel (alpha ~ 0.98)
  const float alpha = 0.98f;
  pitch_deg = alpha * pitch_deg + (1 - alpha) * pitch_acc;
  roll_deg  = alpha * roll_deg  + (1 - alpha) * roll_acc;

  // ----- HIT DETECTOR -----
  // Magnitude of acceleration in g
  float a_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  // Update slow EMA baseline
  a_mag_ema = (1.0f - A_EMA_ALPHA) * a_mag_ema + A_EMA_ALPHA * a_mag;

  // Spike above baseline
  float spike_g = a_mag - a_mag_ema;

  // Optional downward check
  bool downwardish = (az_g < 1.0f + DOWN_G_EXTRA);

  // Decide hit
  bool ready = (now - last_hit_ms) > HIT_REFRACT_MS;
  if (ready && (spike_g > HIT_THRESH_G) && downwardish) {
    last_hit_ms = now;
    last_spike_g = spike_g;

    // Map orientation to a zone at the moment of impact
    const char* zone = mapZone(roll_deg, pitch_deg);
    int vel = spikeToMidi(spike_g);

    Serial.printf("HIT: %s (vel=%d)  roll=%.1f pitch=%.1f spike=%.2fg\n",
                  zone, vel, roll_deg, pitch_deg, spike_g);
  }

  // ---- optional verbose stream for tuning ----
  if (VERBOSE_STREAM) {
    Serial.printf(
      "A[g]=[%6.3f %6.3f %6.3f] |a|=%.3f ema=%.3f sp=%.3f  "
      "G[dps]=[%7.2f %7.2f %7.2f]  T=%5.2fC  R=%.1f P=%.1f\n",
      ax_g, ay_g, az_g, a_mag, a_mag_ema, spike_g,
      gx_dps, gy_dps, gz_dps, temp_c, roll_deg, pitch_deg
    );
  }

  delay(5); // higher rate helps hit timing; still cheap on ESP32
}
