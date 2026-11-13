/**
 * ============================================================================
 * ESP32 Autonomous Aquaculture Robot Firmware (Production Hardware Release)
 * ============================================================================
 * Target MCU : ESP32-WROOM-32 / ESP32-S3 (Arduino Core for ESP32)
 * Serial Baud: 115200 bps (8N1, JSON telemetry & command protocol over UART0)
 *
 * Connected Hardware Peripherals:
 *   1. HX711 24-Bit Load Cell Amplifier : DOUT = GPIO 16, PD_SCK = GPIO 4
 *   2. Analog pH Sensor (0-3.3V BNC)    : ADC1_CH6 = GPIO 34
 *   3. Analog Dissolved Oxygen Sensor   : ADC1_CH7 = GPIO 35
 *   4. Analog Turbidity Sensor (NTU)    : ADC1_CH4 = GPIO 32
 *   5. DS18B20 Waterproof Temp Sensor   : OneWire  = GPIO 27 (4.7k pull-up)
 *   6. Feeder Dispenser Servo + Valve   : PWM = GPIO 18, Relay = GPIO 19, LED = GPIO 2
 *
 * Digital Signal Conditioning & Noise Suppression Pipeline:
 *   - 30x Oversampling per ADC channel
 *   - Sorting & Trimmed Mean (discards bottom 20% [6] and top 20% [6] outliers)
 *   - 1D Kalman Filter + Low-Pass Exponential Moving Average (EMA)
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS & CONSTANTS
// ============================================================================
static const uint8_t PIN_HX711_DOUT      = 16;  // HX711 Data Out
static const uint8_t PIN_HX711_SCK       = 4;   // HX711 Serial Clock
static const uint8_t PIN_PH_ADC          = 34;  // ADC1_CH6 (Input Only, Low Noise)
static const uint8_t PIN_DO_ADC          = 35;  // ADC1_CH7 (Input Only, Low Noise)
static const uint8_t PIN_TURBIDITY_ADC   = 32;  // ADC1_CH4
static const uint8_t PIN_ONEWIRE_TEMP    = 27;  // DS18B20 1-Wire Data Bus
static const uint8_t PIN_FEEDER_SERVO    = 18;  // Dispenser Gate Servo PWM
static const uint8_t PIN_VALVE_RELAY     = 19;  // Redundant Solenoid/Relay Enable
static const uint8_t PIN_STATUS_LED      = 2;   // Onboard Diagnostic LED

// ESP32 LEDC Servo PWM Configuration (50 Hz, 16-bit timer)
static const uint8_t  SERVO_LEDC_CHANNEL = 0;
static const uint32_t SERVO_PWM_FREQ_HZ  = 50;
static const uint8_t  SERVO_PWM_RES_BITS = 16;
static const uint16_t SERVO_PULSE_CLOSED_US = 1000;  // 0 deg (Valve Closed)
static const uint16_t SERVO_PULSE_OPEN_US   = 1900;  // 90 deg (Valve Open)

// Timing Constants (Non-blocking millis() scheduler)
static const uint32_t TELEMETRY_INTERVAL_MS = 50;   // 20 Hz Serial JSON output
static const uint32_t DS18B20_CONV_TIME_MS  = 750;  // 12-bit DS18B20 conversion window

// Sensor Calibration Constants
static const float HX711_CALIBRATION_SCALE = 420.5f;  // Raw counts per gram
static const float ADC_VREF_MV             = 3300.0f; // 3.3V reference in mV
static const float ADC_MAX_COUNTS          = 4095.0f; // 12-bit SAR ADC

// ============================================================================
// 2. DEDICATED NOISE REDUCTION CLASS: OVERSAMPLING + TRIMMED MEAN + KALMAN/EMA
// ============================================================================
class RobustSensorFilter {
public:
  static const size_t OVERSAMPLE_COUNT = 30;
  static const size_t TRIM_COUNT       = 6;  // 20% of 30 samples = 6 low + 6 high

  RobustSensorFilter(float ema_alpha = 0.28f,
                     float kalman_q  = 0.005f,
                     float kalman_r  = 0.08f,
                     float init_val  = 0.0f)
      : alpha_(ema_alpha),
        q_(kalman_q),
        r_(kalman_r),
        x_hat_(init_val),
        p_cov_(1.0f),
        ema_val_(init_val),
        initialized_(false) {}

  void reset(float value) {
    x_hat_       = value;
    p_cov_       = 0.1f;
    ema_val_     = value;
    initialized_ = true;
  }

  /**
   * Acquires 30 analog samples from the specified ESP32 ADC pin, sorts them,
   * discards the top 20% and bottom 20% outliers (Trimmed Mean), and applies
   * the 1D Kalman + Exponential Moving Average filter.
   * Also outputs the single-shot raw voltage (mV) for raw vs. filtered comparison.
   */
  float readFilteredMillivolts(uint8_t adc_pin, float* out_raw_mv = nullptr) {
    float samples[OVERSAMPLE_COUNT];
    for (size_t i = 0; i < OVERSAMPLE_COUNT; ++i) {
      uint16_t raw_adc = analogRead(adc_pin);
      samples[i] = adcCountsToLinearizedMillivolts(raw_adc);
      delayMicroseconds(40);
    }

    if (out_raw_mv != nullptr) {
      *out_raw_mv = samples[0];
    }

    float trimmed_mean_mv = computeTrimmedMean(samples, OVERSAMPLE_COUNT, TRIM_COUNT);
    return updateFilter(trimmed_mean_mv);
  }

  /**
   * Computes Trimmed Mean on an arbitrary array of samples (sorts in-place,
   * removes bottom `trim_each_side` and top `trim_each_side` samples) and
   * passes the result through the Kalman + EMA filter.
   */
  float processSampleArray(float* samples, size_t count, size_t trim_each_side) {
    float trimmed_mean = computeTrimmedMean(samples, count, trim_each_side);
    return updateFilter(trimmed_mean);
  }

  /**
   * Updates the 1D Kalman + Low-Pass EMA filter with a new pre-conditioned value.
   */
  float updateFilter(float measurement) {
    if (!initialized_) {
      reset(measurement);
      return ema_val_;
    }

    // Stage 1: 1D Kalman Filter update
    float p_pred      = p_cov_ + q_;
    float kalman_gain = p_pred / (p_pred + r_);
    x_hat_            = x_hat_ + kalman_gain * (measurement - x_hat_);
    p_cov_            = (1.0f - kalman_gain) * p_pred;

    // Stage 2: Low-Pass Exponential Moving Average (EMA)
    ema_val_ = (alpha_ * x_hat_) + ((1.0f - alpha_) * ema_val_);
    return ema_val_;
  }

  float getFilteredValue() const { return ema_val_; }

  static float computeTrimmedMean(float* arr, size_t n, size_t trim_k) {
    // In-place Insertion Sort (deterministic & fast for N=30 on ESP32)
    for (size_t i = 1; i < n; ++i) {
      float key = arr[i];
      int j = static_cast<int>(i) - 1;
      while (j >= 0 && arr[j] > key) {
        arr[j + 1] = arr[j];
        j--;
      }
      arr[j + 1] = key;
    }

    if (2 * trim_k >= n) {
      return arr[n / 2];
    }

    float sum = 0.0f;
    size_t valid_count = 0;
    for (size_t i = trim_k; i < (n - trim_k); ++i) {
      sum += arr[i];
      valid_count++;
    }
    return (valid_count > 0) ? (sum / static_cast<float>(valid_count)) : arr[n / 2];
  }

  static float adcCountsToLinearizedMillivolts(uint16_t counts) {
    // Compensates ESP32 12-bit SAR ADC 11dB attenuation non-linearity
    if (counts == 0) return 0.0f;
    if (counts >= 4095) return ADC_VREF_MV;
    float norm = static_cast<float>(counts) / ADC_MAX_COUNTS;
    float linear_mv = (0.015f + 0.985f * norm) * ADC_VREF_MV;
    return linear_mv;
  }

private:
  float alpha_;
  float q_;
  float r_;
  float x_hat_;
  float p_cov_;
  float ema_val_;
  bool  initialized_;
};

// ============================================================================
// 3. NON-BLOCKING HX711 24-BIT LOAD CELL DRIVER
// ============================================================================
class HX711Driver {
public:
  HX711Driver(uint8_t dout_pin, uint8_t sck_pin, float scale)
      : dout_pin_(dout_pin),
        sck_pin_(sck_pin),
        scale_(scale),
        tare_offset_(8421500L),
        last_raw_grams_(5000.0f) {}

  void begin() {
    pinMode(dout_pin_, INPUT_PULLUP);
    pinMode(sck_pin_, OUTPUT);
    digitalWrite(sck_pin_, LOW);
  }

  bool isReady() const {
    return (digitalRead(dout_pin_) == LOW);
  }

  int32_t readRaw24Bit() {
    if (!isReady()) {
      return tare_offset_ + static_cast<int32_t>(last_raw_grams_ * scale_);
    }

    uint32_t value = 0;
    noInterrupts();
    for (uint8_t i = 0; i < 24; ++i) {
      digitalWrite(sck_pin_, HIGH);
      delayMicroseconds(1);
      value = (value << 1) | (digitalRead(dout_pin_) & 0x01);
      digitalWrite(sck_pin_, LOW);
      delayMicroseconds(1);
    }
    // 25th pulse selects Channel A, Gain 128
    digitalWrite(sck_pin_, HIGH);
    delayMicroseconds(1);
    digitalWrite(sck_pin_, LOW);
    interrupts();

    // Sign-extend 24-bit two's complement to 32-bit signed integer
    if (value & 0x800000UL) {
      value |= 0xFF000000UL;
    }
    return static_cast<int32_t>(value);
  }

  void tare(size_t samples = 10) {
    int64_t sum = 0;
    size_t count = 0;
    uint32_t start_ms = millis();
    while (count < samples && (millis() - start_ms) < 600) {
      if (isReady()) {
        sum += readRaw24Bit();
        count++;
      }
      delay(5);
    }
    if (count > 0) {
      tare_offset_ = static_cast<int32_t>(sum / static_cast<int64_t>(count));
    }
  }

  float readGrams() {
    int32_t raw_counts = readRaw24Bit();
    float grams = static_cast<float>(raw_counts - tare_offset_) / scale_;
    if (grams < 0.0f) grams = 0.0f;
    last_raw_grams_ = grams;
    return grams;
  }

private:
  uint8_t dout_pin_;
  uint8_t sck_pin_;
  float   scale_;
  int32_t tare_offset_;
  float   last_raw_grams_;
};

// ============================================================================
// 4. NON-BLOCKING ONEWIRE DS18B20 WATERPROOF TEMPERATURE DRIVER
// ============================================================================
class OneWireDS18B20 {
public:
  explicit OneWireDS18B20(uint8_t pin)
      : pin_(pin),
        conversion_started_(false),
        last_conv_start_ms_(0),
        last_temp_c_(25.0f) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
  }

  void updateNonBlocking() {
    uint32_t now = millis();
    if (!conversion_started_) {
      if (resetBus()) {
        writeByte(0xCC);  // Skip ROM
        writeByte(0x44);  // Convert T
        conversion_started_ = true;
        last_conv_start_ms_ = now;
      }
    } else if ((now - last_conv_start_ms_) >= DS18B20_CONV_TIME_MS) {
      if (resetBus()) {
        writeByte(0xCC);  // Skip ROM
        writeByte(0xBE);  // Read Scratchpad
        uint8_t lsb = readByte();
        uint8_t msb = readByte();
        int16_t raw = (static_cast<int16_t>(msb) << 8) | lsb;
        float celsius = static_cast<float>(raw) / 16.0f;
        if (celsius >= -10.0f && celsius <= 65.0f) {
          last_temp_c_ = celsius;
        }
      }
      conversion_started_ = false;
    }
  }

  float getTemperatureC() const { return last_temp_c_; }

private:
  uint8_t  pin_;
  bool     conversion_started_;
  uint32_t last_conv_start_ms_;
  float    last_temp_c_;

  bool resetBus() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    delayMicroseconds(480);
    pinMode(pin_, INPUT_PULLUP);
    delayMicroseconds(70);
    bool presence = (digitalRead(pin_) == LOW);
    delayMicroseconds(410);
    return presence;
  }

  void writeBit(uint8_t bit_val) {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    if (bit_val & 1) {
      delayMicroseconds(6);
      pinMode(pin_, INPUT_PULLUP);
      delayMicroseconds(64);
    } else {
      delayMicroseconds(60);
      pinMode(pin_, INPUT_PULLUP);
      delayMicroseconds(10);
    }
  }

  uint8_t readBit() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    delayMicroseconds(3);
    pinMode(pin_, INPUT_PULLUP);
    delayMicroseconds(10);
    uint8_t r = digitalRead(pin_);
    delayMicroseconds(53);
    return r;
  }

  void writeByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; ++i) {
      writeBit((data >> i) & 0x01);
    }
  }

  uint8_t readByte() {
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      if (readBit()) {
        value |= (1 << i);
      }
    }
    return value;
  }
};

// ============================================================================
// 5. GLOBAL INSTANCES & STATE VARIABLES
// ============================================================================
static HX711Driver        g_hx711(PIN_HX711_DOUT, PIN_HX711_SCK, HX711_CALIBRATION_SCALE);
static OneWireDS18B20     g_ds18b20(PIN_ONEWIRE_TEMP);

static RobustSensorFilter g_weight_filter(0.35f, 0.20f, 8.0f, 5000.0f);
static RobustSensorFilter g_ph_filter(0.25f, 0.002f, 0.06f, 1650.0f);
static RobustSensorFilter g_do_filter(0.25f, 0.004f, 0.08f, 1500.0f);
static RobustSensorFilter g_turb_filter(0.25f, 0.05f, 2.0f, 2100.0f);
static RobustSensorFilter g_temp_filter(0.25f, 0.002f, 0.04f, 25.0f);

static bool     g_valve_open          = false;
static uint32_t g_last_telemetry_ms   = 0;
static char     g_serial_rx_buf[128];
static size_t   g_serial_rx_idx       = 0;

// Weight buffer for rolling trimmed mean on non-blocking HX711 stream
static const size_t HX711_WINDOW_SIZE = 10;
static float        g_hx711_ring[HX711_WINDOW_SIZE];
static size_t       g_hx711_ring_idx  = 0;
static bool         g_hx711_ring_full = false;

// ============================================================================
// 6. ACTUATOR & SENSOR CONVERSION FUNCTIONS
// ============================================================================
static void setFeederValveState(bool open_valve) {
  g_valve_open = open_valve;
  uint16_t pulse_us = open_valve ? SERVO_PULSE_OPEN_US : SERVO_PULSE_CLOSED_US;
  // Convert pulse width in us (out of 20000 us period at 50 Hz) to 16-bit duty
  uint32_t duty = (static_cast<uint32_t>(pulse_us) * 65535UL) / 20000UL;
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
  digitalWrite(PIN_VALVE_RELAY, open_valve ? HIGH : LOW);
  digitalWrite(PIN_STATUS_LED, open_valve ? HIGH : LOW);
}

static float convertMillivoltsToPH(float mv, float temp_c) {
  // Neutral pH 7.0 corresponds to 1650 mV; Nernst slope ~ -59.16 mV/pH at 25C
  float temp_factor = (temp_c + 273.15f) / 298.15f;
  float slope_mv_per_ph = -59.16f * temp_factor;
  float ph = 7.0f + ((mv - 1650.0f) / slope_mv_per_ph);
  if (ph < 0.0f) ph = 0.0f;
  if (ph > 14.0f) ph = 14.0f;
  return ph;
}

static float convertMillivoltsToDO(float mv, float temp_c) {
  // Temperature-compensated Dissolved Oxygen conversion (mg/L)
  // Saturation DO at 25C is ~8.26 mg/L at ~1600 mV calibration voltage
  float sat_do_mg_l = 14.62f - 0.3898f * temp_c + 0.006969f * temp_c * temp_c
                      - 0.00005896f * temp_c * temp_c * temp_c;
  float do_mg_l = (mv / 1600.0f) * sat_do_mg_l;
  return (do_mg_l < 0.0f) ? 0.0f : do_mg_l;
}

static float convertMillivoltsToTurbidityNTU(float mv) {
  // Standard Nephelometric Turbidity Unit (NTU) polynomial curve
  // Clear water (~2500 mV) -> ~0-10 NTU; Murky water (<1200 mV) -> >100 NTU
  float v = mv / 1000.0f;
  if (v >= 2.55f) return 5.0f;
  if (v <= 0.50f) return 300.0f;
  float ntu = -65.0f * v * v + 95.0f * v + 180.0f;
  return (ntu < 0.0f) ? 0.0f : ntu;
}

// ============================================================================
// 7. NON-BLOCKING SERIAL JSON COMMAND PARSER
// ============================================================================
static void processSerialCommandLine(const char* line) {
  // Expected commands: {"valve": 1}, {"valve": 0}, {"tare": 1}
  const char* valve_key = strstr(line, "\"valve\"");
  if (valve_key != nullptr) {
    const char* colon = strchr(valve_key, ':');
    if (colon != nullptr) {
      int cmd_val = atoi(colon + 1);
      setFeederValveState(cmd_val != 0);
    }
  }

  const char* tare_key = strstr(line, "\"tare\"");
  if (tare_key != nullptr) {
    const char* colon = strchr(tare_key, ':');
    if (colon != nullptr && atoi(colon + 1) != 0) {
      g_hx711.tare(10);
      g_weight_filter.reset(0.0f);
    }
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_serial_rx_idx > 0) {
        g_serial_rx_buf[g_serial_rx_idx] = '\0';
        processSerialCommandLine(g_serial_rx_buf);
        g_serial_rx_idx = 0;
      }
    } else if (g_serial_rx_idx < (sizeof(g_serial_rx_buf) - 1)) {
      g_serial_rx_buf[g_serial_rx_idx++] = c;
    } else {
      g_serial_rx_idx = 0;  // Buffer overflow protection
    }
  }
}

// ============================================================================
// 8. ARDUINO SETUP & NON-BLOCKING MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Configure Digital I/O & Actuators
  pinMode(PIN_VALVE_RELAY, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_VALVE_RELAY, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Configure ESP32 ADC resolution & attenuation
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PH_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_DO_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_TURBIDITY_ADC, ADC_11db);

  // Configure Feeder Servo PWM
  ledcSetup(SERVO_LEDC_CHANNEL, SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
  ledcAttachPin(PIN_FEEDER_SERVO, SERVO_LEDC_CHANNEL);
  setFeederValveState(false);

  // Initialize HX711 & DS18B20
  g_hx711.begin();
  g_ds18b20.begin();

  for (size_t i = 0; i < HX711_WINDOW_SIZE; ++i) {
    g_hx711_ring[i] = 5000.0f;
  }

  g_last_telemetry_ms = millis();
}

void loop() {
  // 1. Non-blocking Serial command parsing from Raspberry Pi
  pollSerialCommands();

  // 2. Non-blocking DS18B20 1-Wire temperature state machine
  g_ds18b20.updateNonBlocking();

  // 3. Non-blocking 20 Hz Sensor Sampling, Filtering, and JSON Telemetry
  uint32_t now_ms = millis();
  if ((now_ms - g_last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {
    g_last_telemetry_ms = now_ms;

    // A. Water Temperature (DS18B20 + Kalman/EMA Filter)
    float raw_temp_c  = g_ds18b20.getTemperatureC();
    float filt_temp_c = g_temp_filter.updateFilter(raw_temp_c);

    // B. Analog pH Sensor (30x Oversampling + Trimmed Mean + Kalman/EMA)
    float raw_ph_mv   = 1650.0f;
    float filt_ph_mv  = g_ph_filter.readFilteredMillivolts(PIN_PH_ADC, &raw_ph_mv);
    float raw_ph      = convertMillivoltsToPH(raw_ph_mv, filt_temp_c);
    float filt_ph     = convertMillivoltsToPH(filt_ph_mv, filt_temp_c);

    // C. Analog Dissolved Oxygen Sensor (30x Oversampling + Trimmed Mean + Kalman/EMA)
    float raw_do_mv   = 1500.0f;
    float filt_do_mv  = g_do_filter.readFilteredMillivolts(PIN_DO_ADC, &raw_do_mv);
    float raw_do      = convertMillivoltsToDO(raw_do_mv, filt_temp_c);
    float filt_do     = convertMillivoltsToDO(filt_do_mv, filt_temp_c);

    // D. Analog Turbidity Sensor (30x Oversampling + Trimmed Mean + Kalman/EMA)
    float raw_turb_mv  = 2100.0f;
    float filt_turb_mv = g_turb_filter.readFilteredMillivolts(PIN_TURBIDITY_ADC, &raw_turb_mv);
    float raw_turb     = convertMillivoltsToTurbidityNTU(raw_turb_mv);
    float filt_turb    = convertMillivoltsToTurbidityNTU(filt_turb_mv);

    // E. HX711 Load Cell Weight (Rolling Trimmed Mean + Kalman/EMA)
    float raw_weight_g = g_hx711.readGrams();
    g_hx711_ring[g_hx711_ring_idx] = raw_weight_g;
    g_hx711_ring_idx = (g_hx711_ring_idx + 1) % HX711_WINDOW_SIZE;
    if (g_hx711_ring_idx == 0) {
      g_hx711_ring_full = true;
    }

    float temp_ring_copy[HX711_WINDOW_SIZE];
    size_t active_n = g_hx711_ring_full ? HX711_WINDOW_SIZE : max<size_t>(1, g_hx711_ring_idx);
    for (size_t i = 0; i < active_n; ++i) {
      temp_ring_copy[i] = g_hx711_ring[i];
    }
    size_t trim_k = (active_n >= 5) ? (active_n / 5) : 0;
    float filt_weight_g = g_weight_filter.processSampleArray(temp_ring_copy, active_n, trim_k);

    // F. Transmit JSON Packet over Serial (115200 baud) to Raspberry Pi
    char json_out[320];
    snprintf(
        json_out,
        sizeof(json_out),
        "{\"weight\":%.2f,\"ph\":%.3f,\"do\":%.3f,\"turbidity\":%.2f,\"temp\":%.2f,"
        "\"raw_weight\":%.2f,\"raw_ph\":%.3f,\"raw_do\":%.3f,\"raw_turbidity\":%.2f,"
        "\"raw_temp\":%.2f,\"valve\":%d}\n",
        filt_weight_g,
        filt_ph,
        filt_do,
        filt_turb,
        filt_temp_c,
        raw_weight_g,
        raw_ph,
        raw_do,
        raw_turb,
        raw_temp_c,
        g_valve_open ? 1 : 0);
    Serial.print(json_out);
  }
}
