/*********************************************************************************************************
 * ESP32S3-Zero_Cubean_Roaster Control_v2.2 (For Artisan + HiBean)
 *
 * This is the complete, corrected code for controlling a Cubean roaster with an ESP32-S3.
 * It merges a feature-rich controller base with the reverse-engineered Cubean protocol.
 *
 * KEY FEATURES:
 * - Communication: Standard UART (Serial1) for sending and receiving 11-byte FE EF packets.
 * - Temperature Decoding: Implements the verified three-formula system for high accuracy.
 * - Checksum Calculation: Dynamically calculates the correct checksum for all outgoing commands.
 * - Connectivity: Retains full BLE, WiFi AP/STA Mode, WebSockets, and TCP Server for Artisan.
 * - Advanced Control: Retains PID auto-switching, web UI, and auto-shutdown features.
 *
 * HARDWARE:
 * - Board: WAVESHARE_ESP32_S3
 * - UART TX_PIN -> Cubean RX: 20
 * - UART RX_PIN <- Cubean TX: 19
 * - UART RX_PIN_PANEL <- Panel TX: 21
 *********************************************************************************************************/ 
// dummy
 #include <Arduino.h>
 #include <ArduinoJson.h>
 #include <AsyncTCP.h>
 #include <ESPAsyncWebServer.h>
 #include <ESPmDNS.h>
 #include <FastLED.h>
 #include <NimBLEDevice.h>
 #include <PID_v1.h>
 #include <Preferences.h>
 #include <Update.h>
 #include <WiFi.h>
 
 #include "driver/uart.h"
 
 #ifndef SERIAL_DEBUG
 #define SERIAL_DEBUG 0
 #endif
 
 #if SERIAL_DEBUG == 1
 #include <WebSerial.h>
 #endif

 // -----------------------------------------------------------------------------
 // Debug Settings
 // -----------------------------------------------------------------------------
 #if SERIAL_DEBUG == 1
 static bool g_webSerialInit = false;
 inline bool webSerialHasClient() {
  return g_webSerialInit && (WebSerial.getConnectionCount() > 0);
}

 #define D_print(...) if (webSerialHasClient()) WebSerial.print(__VA_ARGS__)
 #define D_println(...) if (webSerialHasClient()) WebSerial.println(__VA_ARGS__)
 #define D_printf(...) if (webSerialHasClient()) WebSerial.printf(__VA_ARGS__)
 #else
 #define D_print(...)
 #define D_println(...)
 #define D_printf(...)
 #endif
 
 // -----------------------------------------------------------------------------
 // Pin & Core Definitions
 // -----------------------------------------------------------------------------
 const int TX_PIN = 20;       // UART TX to Cubean RX
 const int RX_PIN = 19;       // UART RX from Cubean TX
 const int RX_PIN_PANEL = 21; // UART RX from Panel TX
 const int LED_PIN = 48;
 const String boardID_BLE = String("CUBEAN_ESP32");
 
 // -----------------------------------------------------------------------------
 // FastLED Configuration
 // -----------------------------------------------------------------------------
 #define NUM_LEDS 1
 #define LED_TYPE WS2812B
 #define COLOR_ORDER GRB
 CRGB leds[NUM_LEDS];
 const CRGB LED_RED = CRGB::Red;
 const CRGB LED_GREEN = CRGB::Green;
 const CRGB LED_BLUE = CRGB::Blue;
 
 // -----------------------------------------------------------------------------
 // BLE UUIDs for Nordic UART Service (NUS)
 // -----------------------------------------------------------------------------
 #define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
 #define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
 #define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
 
 // Queue size
 #define BLE_Q_LEN 10
 
 struct BleMsg {
   uint16_t len;
   char buf[64];
 };
 static StaticQueue_t bleQCtrl;
 alignas(4) static uint8_t bleQStorage[BLE_Q_LEN * sizeof(BleMsg)];
 static QueueHandle_t bleQueue;
 
 // -----------------------------------------------------------------------------
 // Roaster UART
 // -----------------------------------------------------------------------------
 static constexpr uart_port_t ROASTER_UART = UART_NUM_1;
 static constexpr int ROASTER_RX_RING = 1024;
 static constexpr int ROASTER_TX_RING = 256;
 static constexpr int ROASTER_READ_CHUNK = 128;
 static constexpr uart_port_t PANEL_UART = UART_NUM_2;
 static constexpr int PANEL_RX_RING = 512;
 static constexpr int PANEL_READ_CHUNK = 128;
 static constexpr uint8_t H1 = 0xFE;
 static constexpr uint8_t H2 = 0xEF;
 static constexpr size_t FRAME_LEN = 11;
 
 static inline uint8_t calc_checksum_0_to_9(const uint8_t *f) {
   uint8_t s = 0;
   for (int i = 0; i < 10; ++i)
     s += f[i];
   return s;
 }
 
 // -----------------------------------------------------------------------------
 // WiFi & Web Server Configuration
 // -----------------------------------------------------------------------------
 AsyncWebServer server(80);
 AsyncWebSocket webSocket("/ws");
 const char *wifiAPSSID = "Cubean_Roaster";
 const char *wifiAPPass = ""; // No password for AP mode
 Preferences preferences;
 
 static bool otaActive = false;
 static unsigned long otaStartTime = 0;
 static const unsigned long OTA_TIMEOUT_MS = 300000; // 5 minutes timeout
 static bool restartPending = false;
 static unsigned long restartTime = 0;
 
 // -----------------------------------------------------------------------------
 // Global Variables
 // -----------------------------------------------------------------------------
 #ifndef FW_VERSION
 #define FW_VERSION "Cubean_Control_v2.2"
 #endif
 constexpr char firmWareVersion[] = FW_VERSION;
 
 // Roaster State
 float temp = 0.0;
 char CorF = 'C';
 uint8_t currentFan = 0;
 uint8_t currentHeat = 0;
 uint8_t currentDrum = 0;
 bool isCooling = false;
 uint8_t filterValue = 5; // Filter level (1-5)
 
 static bool autoCoolingArmed = false;   // set true when heat > 0 at least once
 static bool autoCoolingUsed  = false;   // set true after an auto-cooling cycle completes (hit low temp)
 
 uint8_t messageToSend[11] = {
     0xFE, 0xEF,
     0x05, // Byte 2: Filter
     0x00, // Unknown, Always 0
     0x00, // Byte 4: Heater
     0x00, // Byte 5: Fan
     0x00, // Byte 6: Fixed value (e.g., 0x02)
     0x00, // Byte 7: State (Idle, Roasting, Cooling)
     0xAA, 0x55,
     0x00 // Checksum placeholder
 };
 
 enum class RoasterState : uint8_t {
   Idle = 0x0,
   Active = 0x5,
   ActiveCoolingAuto = 0x6,
   BeanCoolingLowTemp = 0x8,
   BeanCoolingHighTemp = 0xE,
 };
 
 RoasterState roasterState = RoasterState::Idle;
 uint32_t lastMessageSentToRoasterTime = 0;
 
 // Panel State
 uint8_t panelState = 0x20;
 bool havePanelState = false;
 uint32_t lastPanelSeenMs = 0;
 const uint32_t PANEL_TTL_MS = 2000; // 2 second
 uint8_t panelPacket[11];            // Store the last panel packet for correlation
 
 // PID variables
 double pInput, pOutput, pSetpoint = 0.0;
 double Kp = 12.0, Ki = 1.5, Kd = 5.0;
 int pMode = P_ON_M;
 PID pid(&pInput, &pOutput, &pSetpoint, Kp, Ki, Kd, pMode, DIRECT);
 unsigned long lastArtisanCommandTime = 0;
 const unsigned long PID_AUTOSWITCH_TIMEOUT = 3000; // 3 seconds
 uint8_t manualHeatLevel = 0;
 
 // BLE variables
 uint8_t bleMtu = 158;
 NimBLEServer *pServer = nullptr;
 NimBLECharacteristic *pTxCharacteristic = nullptr;
 
 // Auto-shutdown variables
 const double COOL_SHUTDOWN_TEMP_C = 50.0;
 
 // Forward Declarations
 void parseAndExecuteCommands(String input, bool fromBLE = false);
 void sendRoasterMessage();
 void getRoasterMessage();
 
 // -----------------------------------------------------------------------------
 // CUBEAN COMMUNICATION LOGIC
 // -----------------------------------------------------------------------------
 
 static inline void autoCoolingRearmOnHeat() {  // call when heater > 0
   autoCoolingArmed = true;
   autoCoolingUsed  = false;
 }
 static inline void autoCoolingMarkUsed() {      // call when auto-cool cycle completes
   autoCoolingUsed = true;
 }
 static inline void autoCoolingReset() {          // boot/shutdown hard reset
   autoCoolingArmed = false;
   autoCoolingUsed  = false;
 }
 
 // ---------- ADC resolution ----------
 // 12-bit ADC
 static const int ADC_MAX = 4095;
 // Hysteresis around 80 °C
 const float T_LOW = 78.f, T_HIGH = 80.f;
 
 static inline float ln_ratio_from_raw(uint16_t raw) {
   if (raw < 1) raw = 1;
   if (raw > ADC_MAX - 1) raw = ADC_MAX - 1;
   return logf((float)raw / (float)(ADC_MAX - raw));
 }
 
 static inline float eval_poly3(float a, float b, float c, float x) {
   return a + b * x + c * (x * x * x);
 }
 
 static inline float eval_asin(float value) {
   const float arg = constrain(0.00063224f * value - 1.4507f, -1.0f, 1.0f);
   return -66.198f * asin(arg) + 162.93f;
 }
 
 // ---------- Coefficients per family ----------
 // LOW family (0x2* and 0x6* ~ below 80°C)
 static const float LOW_A = 74.49f, LOW_B = -24.65f, LOW_C = -0.24f;
 // HIGH family (0x3* and 0x7* ~ above 80°C)
 static const float HIGH_A = 171.917f, HIGH_B = -38.975f, HIGH_C = 0.536f;
 
 // ---------- State handling (family + step) ----------
 // Helpers: treat 0x2* and 0x6* as LOW family; 0x3* and 0x7* as HIGH family
 static inline bool is_low_family(uint8_t s) {
   uint8_t f = s & 0xF0;
   return (f == 0x00 || f == 0x20 || f == 0x60);
 }
 static inline bool is_high_family(uint8_t s) {
   uint8_t f = s & 0xF0;
   return (f == 0x30 || f == 0x70);
 }
 
 // ---------- Family-based decode ----------
 // ---------- If panel state is unavailable, auto-select family with 80 °C hysteresis ----------
 enum class TempFamily : uint8_t { LOW_FAMILY = 0, HIGH_FAMILY = 1 };
 
 struct TempAutoCtx {
   float prevTempC = NAN;
   TempFamily lastFam = TempFamily::LOW_FAMILY;
 };
 TempAutoCtx tempCtx;
 
 // Choose curve by panel state family (now supports 0x2*/0x3* and 0x6*/0x7*)
 static inline float tempC_from_raw_stateFamily(uint16_t raw, uint8_t panel_state) {
   const float x = ln_ratio_from_raw(raw);
 
   if (is_low_family(panel_state)) return eval_poly3(LOW_A, LOW_B, LOW_C, x);
   if (is_high_family(panel_state)) return eval_asin(raw);
 
   // Fallback: pick the one closer to 80 °C
   const float yL = eval_poly3(LOW_A, LOW_B, LOW_C, x);
   const float yH = eval_poly3(HIGH_A, HIGH_B, HIGH_C, x);
   return (fabsf(yL - 80.f) < fabsf(yH - 80.f)) ? yL : yH;
 }
 
 // Improved auto if panel state temporarily missing.
 // Keeps hysteresis and honors the early page-flip rule via ro_b6.
 static inline float tempC_from_raw_auto(uint16_t raw, TempAutoCtx &ctx) {
   // Guard bad raw
   if (raw == 0 || raw >= ADC_MAX) return isnan(ctx.prevTempC) ? 25.0f : ctx.prevTempC;
 
   // Curves
   const float x  = ln_ratio_from_raw(raw);
   const float yL = eval_poly3(LOW_A, LOW_B, LOW_C, x);
   const float yH = eval_asin(raw);
 
   // First sample -> start LOW
   if (isnan(ctx.prevTempC)) {
     ctx.lastFam  = TempFamily::LOW_FAMILY;
     ctx.prevTempC = yL;
     return ctx.prevTempC;
   }
 
   // ---- Decide which family we "prefer" this sample ----
   constexpr float MARGIN = 2.0f;          // °C extra hysteresis
   TempFamily prefer = ctx.lastFam;
 
   if (ctx.prevTempC <= (T_LOW - MARGIN))        prefer = TempFamily::LOW_FAMILY;
   else if (ctx.prevTempC >= (T_HIGH + MARGIN))  prefer = TempFamily::HIGH_FAMILY;
   else {
     // In the boundary band: choose the curve that causes the smaller jump
     const float dL = fabsf(yL - ctx.prevTempC);
     const float dH = fabsf(yH - ctx.prevTempC);
     prefer = (dH < dL) ? TempFamily::HIGH_FAMILY : TempFamily::LOW_FAMILY;
   }
 
   // ---- Debounce family switching ----
   static TempFamily pendingFam = TempFamily::LOW_FAMILY;
   static uint8_t    pendingCnt = 0;
   static uint32_t   lastSwitchMs = 0;
 
   const uint32_t now = millis();
   const bool cooldownOver = (now - lastSwitchMs) >= 1500; // 1.5 s
 
   if (prefer != ctx.lastFam) {
     if (pendingFam != prefer) { pendingFam = prefer; pendingCnt = 1; }
     else if (pendingCnt < 2)  { ++pendingCnt; }
 
     if (pendingCnt >= 2 && cooldownOver) {
       ctx.lastFam = prefer;
       lastSwitchMs = now;
       pendingCnt = 0;
     }
   } else {
     pendingCnt = 0; // stable
   }
 
   // ---- Evaluate with the (possibly updated) family ----
   float y = (ctx.lastFam == TempFamily::LOW_FAMILY) ? yL : yH;
 
   // ---- Rate limit & light smoothing ----
   static uint32_t lastMs = 0;
   const uint32_t dtMs = (lastMs == 0) ? 0 : (now - lastMs);
   lastMs = now;
 
   // Allow up to ~4 °C/s (tune for your sensor dynamics)
   constexpr float RATE_LIMIT_C_PER_S = 4.0f;
   const float maxStep = (dtMs ? RATE_LIMIT_C_PER_S * (dtMs / 1000.0f) : 999.0f);
 
   float delta = y - ctx.prevTempC;
   if (delta >  maxStep) y = ctx.prevTempC + maxStep;
   if (delta < -maxStep) y = ctx.prevTempC - maxStep;
 
   // EMA smoothing (light)
   constexpr float ALPHA = 0.30f; // 0..1 (higher = snappier)
   y = ctx.prevTempC + ALPHA * (y - ctx.prevTempC);
 
   // Clamp to sane bounds
   if (y < -10.0f) y = -10.0f;
   if (y > 300.0f) y = 300.0f;
 
   ctx.prevTempC = y;
   return y;
 }
 
 /**
  * @brief Checks if the panel is connected.
  * @return True if the panel is connected, false otherwise.
  */
 static inline bool isPanelConnected() {
   return havePanelState && (millis() - lastPanelSeenMs <= PANEL_TTL_MS);
 }
 
 /**
  * @brief Decodes temperature from Cubean data using the three-formula system.
  * @param packet The complete 11-byte packet from the roaster.
  * @return Temperature in Celsius.
  */
 float decodeCubeanTemp(const uint8_t *packet) {
   // Roaster packet structure: FE EF [2]=MSB [3]=LSB ... [9 data bytes]
   const uint16_t raw = (uint16_t(packet[2]) << 8) | packet[3];
 
   if (isPanelConnected()) {
     return tempC_from_raw_stateFamily(raw, panelState);
   } else {
     return tempC_from_raw_auto(raw, tempCtx);
   }
 }
 
 // Choose family nibble for *roasting* (0x?D) via temp hysteresis
 static inline uint8_t roast_family_from_temp(float tempC) {
   if (tempC <= T_LOW) return 0x20;
   if (tempC >= T_HIGH) return 0x30;
   return 0x20; // 0x3* = HIGH, 0x2* = LOW
 }
 
 // Define the current state of the roaster
 // low nibble (step): 0x00 idle, 0x05 active, 0x06 max cooling, 0x0E bean cooling
 static RoasterState defineRoasterState(float tempC, uint8_t heaterPct, uint8_t fanPct, bool drumOn) {
   // Cooling takes priority, disables heat and fan
   // 0x0E when roast profiles is on, when idle 0x08
   // To simulate the behavior of the roaster, we need check if the temp is above the threshold
   if (isCooling && tempC >  COOL_SHUTDOWN_TEMP_C) return RoasterState::BeanCoolingHighTemp;
   if (isCooling && tempC <= COOL_SHUTDOWN_TEMP_C) return RoasterState::BeanCoolingLowTemp;
 
   // If we were auto-cooling and we reached the low threshold, mark the cycle as used.
   // (roasterState still holds the previous state when this is called)
   if (roasterState == RoasterState::ActiveCoolingAuto && tempC <= COOL_SHUTDOWN_TEMP_C) {
     autoCoolingMarkUsed();   // don't auto-cool again until heat is applied again
   }
 
   // Any active action?
   if (heaterPct > 0 || fanPct > 0 || drumOn) {
     return RoasterState::Active;
   }
 
   // Auto-cooling: only once per heating cycle.
   // Must be ARMED by prior heating, and not yet USED.
   if (heaterPct == 0 && tempC > COOL_SHUTDOWN_TEMP_C && autoCoolingArmed && !autoCoolingUsed) {
     return RoasterState::ActiveCoolingAuto;
   }
 
   return RoasterState::Idle;
 }
 
 void roaster_uart_init() {
   uart_config_t cfg = {
       .baud_rate = 9600,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
 #if SOC_UART_SUPPORT_REF_TICK
       .source_clk = UART_SCLK_APB,
 #endif
   };
   ESP_ERROR_CHECK(uart_param_config(ROASTER_UART, &cfg));
   ESP_ERROR_CHECK(uart_set_pin(ROASTER_UART, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
 
   // RX+TX rings; no event queue
   ESP_ERROR_CHECK(uart_driver_install(ROASTER_UART, ROASTER_RX_RING, ROASTER_TX_RING, 0, nullptr, 0));
 
   // Fewer wakeups, low tail latency
   uart_set_rx_full_threshold(ROASTER_UART, 22); // ~2 frames
   uart_set_rx_timeout(ROASTER_UART, 3);         // ~3 char idle (~3 ms)
 }
 
 void panel_uart_init() {
   uart_config_t cfg = {
       .baud_rate = 9600,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
 #if SOC_UART_SUPPORT_REF_TICK
       .source_clk = UART_SCLK_APB,
 #endif
   };
   ESP_ERROR_CHECK(uart_param_config(PANEL_UART, &cfg));
   // RX only; leave TX/CTS/RTS unchanged
   ESP_ERROR_CHECK(uart_set_pin(PANEL_UART,
                                UART_PIN_NO_CHANGE,   // TX (unused)
                                RX_PIN_PANEL,         // RX (your pin 21)
                                UART_PIN_NO_CHANGE,   // RTS
                                UART_PIN_NO_CHANGE)); // CTS
 
   // RX ring only; no TX buffer, no event queue
   ESP_ERROR_CHECK(uart_driver_install(PANEL_UART, PANEL_RX_RING, 0, 0, nullptr, 0));
 
   // Fewer wakeups, low tail latency
   uart_set_rx_full_threshold(PANEL_UART, 22);
   uart_set_rx_timeout(PANEL_UART, 3);
 }
 
 // Shared FE EF ... checksum frame reader (non-blocking, skip if incomplete)
 struct UartFsm {
   size_t idx = 0;
   uint8_t frame[FRAME_LEN];
 };
 
 static bool feef_poll_frame(uart_port_t port, UartFsm &fsm, uint8_t out[FRAME_LEN], int read_chunk) {
   size_t avail = 0;
   uart_get_buffered_data_len(port, &avail);
   if (avail < (FRAME_LEN - fsm.idx)) return false; // not enough to finish -> skip
 
   uint8_t tmp[128];
   const int toRead = (int)min(avail, (size_t)min((size_t)read_chunk, sizeof(tmp)));
   int n = uart_read_bytes(port, tmp, toRead, 0); // no wait
   if (n <= 0) return false;
 
   for (int i = 0; i < n; ++i) {
     const uint8_t b = tmp[i];
 
     if (fsm.idx == 0) {
       if (b == H1) {
         fsm.frame[fsm.idx++] = b;
       }
     } else if (fsm.idx == 1) {
       if (b == H2) {
         fsm.frame[fsm.idx++] = b;
       } else {
         // overlap resync: treat as potential new H1
         fsm.idx = (b == H1) ? 1 : 0;
         if (fsm.idx == 1) fsm.frame[0] = H1;
       }
     } else {
       fsm.frame[fsm.idx++] = b;
       if (fsm.idx == FRAME_LEN) {
         const bool ok = (fsm.frame[0] == H1 && fsm.frame[1] == H2 && calc_checksum_0_to_9(fsm.frame) == fsm.frame[FRAME_LEN - 1]);
         fsm.idx = 0;
         if (ok) {
           memcpy(out, fsm.frame, FRAME_LEN);
           return true;
         }
       }
     }
   }
   return false;
 }
 
 static UartFsm roasterFsm;
 static UartFsm panelFsm;
 
 static inline bool roaster_send_frame(const uint8_t f[FRAME_LEN], TickType_t waitTicks = pdMS_TO_TICKS(20)) {
   int wrote = uart_write_bytes(ROASTER_UART, (const char *)f, FRAME_LEN);
   if (wrote != (int)FRAME_LEN) return false;
   (void)uart_wait_tx_done(ROASTER_UART, waitTicks); // ~11.5 ms @9600 for 11B
   return true;
 }
 
 static void processRoasterFrame(const uint8_t buffer[11]) {
   float newTemp = decodeCubeanTemp(buffer);
 
   // Plausibility
   if (abs(newTemp - temp) < 50 || temp == 0.0) {
     temp = newTemp;
   }
 
   // Throttle to 1 Hz and avoid building strings if nobody is listening
   constexpr uint32_t WS_DEBUG_THROTTLE_MS = 1000;
   static uint32_t lastWsSendMs = 0;
   const uint32_t now = millis();
   if ((now - lastWsSendMs) >= WS_DEBUG_THROTTLE_MS) {
     if (isPanelConnected()) {
       D_printf("DEBUG: [%u ms] | Roaster: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | "
                "Panel: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | "
                "ESP: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | Temp: %.1f°C",
                now, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10],
                panelPacket[0], panelPacket[1], panelPacket[2], panelPacket[3], panelPacket[4], panelPacket[5], panelPacket[6], panelPacket[7],
                panelPacket[8], panelPacket[9], panelPacket[10], messageToSend[0], messageToSend[1], messageToSend[2], messageToSend[3],
                messageToSend[4], messageToSend[5], messageToSend[6], messageToSend[7], messageToSend[8], messageToSend[9], messageToSend[10], temp);
     } else {
       D_printf("DEBUG: [%u ms] | Roaster: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | "
                "Panel: N/A | ESP: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | "
                "Temp: %.1f°C",
                now, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10],
                messageToSend[0], messageToSend[1], messageToSend[2], messageToSend[3], messageToSend[4], messageToSend[5], messageToSend[6],
                messageToSend[7], messageToSend[8], messageToSend[9], messageToSend[10], temp);
     }
 
     lastWsSendMs = now;
   }
 }
 
 /**
  * @brief Sends a command packet to the Cubean roaster with a valid checksum.
  */
 void sendRoasterMessage() {
   roasterState = defineRoasterState(temp, currentHeat, currentFan, (currentDrum > 0));
   if (!isPanelConnected() && millis() - lastMessageSentToRoasterTime > 250) {
     const uint8_t fam = roast_family_from_temp(temp);
     uint8_t fan = roasterState == RoasterState::ActiveCoolingAuto ? 100 : currentFan;
     messageToSend[2] = filterValue;
     messageToSend[4] = currentHeat;
     messageToSend[5] = fan;
     messageToSend[6] = fan > 0 ? 0x02 : currentDrum;
     messageToSend[7] = (fam | (uint8_t)roasterState);
 
     uint16_t checksum = 0;
     for (int i = 0; i < 10; i++) {
       checksum += messageToSend[i];
     }
     messageToSend[10] = checksum & 0xFF;
 
     // Serial1.write(messageToSend, sizeof(messageToSend));
     roaster_send_frame(messageToSend);
     lastMessageSentToRoasterTime = millis();
   }
 }
 
 /**
  * @brief Reads and parses a status packet from the Cubean roaster.
  */
 void getRoasterMessage() {
   uint8_t frame[FRAME_LEN];
   if (feef_poll_frame(ROASTER_UART, roasterFsm, frame, ROASTER_READ_CHUNK)) {
     processRoasterFrame(frame);
   }
 }
 
 /**
  * @brief Reads and parses a status packet from the Cubean panel with proper alignment.
  */
 void getPanelMessage() {
   uint8_t f[FRAME_LEN];
   if (feef_poll_frame(PANEL_UART, panelFsm, f, PANEL_READ_CHUNK)) {
     // Same side effects you already had
     panelState = f[7];
     memcpy(panelPacket, f, FRAME_LEN);
     lastPanelSeenMs = millis();
     havePanelState = true;
   }
 }
 
 // =============================================================================
 // LED & SYSTEM STATUS
 // =============================================================================
 bool deviceConnected();
 
 void setRGBColor(CRGB color) {
   leds[0] = color;
   FastLED.show();
 }
 
 void handleLED() {
   static unsigned long lastBlinkTime = 0;
   static bool ledState = false;
 
   if (deviceConnected()) {
     setRGBColor(LED_BLUE);
   } else if (WiFi.status() == WL_CONNECTED) {
     if (millis() - lastBlinkTime > 1000) {
       lastBlinkTime = millis();
       ledState = !ledState;
       setRGBColor(ledState ? LED_GREEN : CRGB::Black);
     }
   } else { // AP Mode
     if (millis() - lastBlinkTime > 250) {
       lastBlinkTime = millis();
       ledState = !ledState;
       setRGBColor(ledState ? LED_RED : CRGB::Black);
     }
   }
 }
 
 void shutdown() {
   currentFan = 0;
   currentHeat = 0;
   currentDrum = 0;
   isCooling = false;
   autoCoolingReset();
   sendRoasterMessage(); // Send shutdown command
   D_println("SYSTEM SHUTDOWN: All controls set to 0.");
 }
 
 // =============================================================================
 // PID & COMMAND HANDLING
 // =============================================================================
 void handleOT1(uint8_t value);
 void handleVENT(uint8_t value);
 void handleHEAT(uint8_t value);
 
 void applyArtisanPID(String command) {
   // Example: "PID:10;50;0;0;100" -> OT1=10, OT2=50, SV=0
   int first = command.indexOf(':');
   String values = command.substring(first + 1);
 
   int h, v, sv;
   sscanf(values.c_str(), "%d;%d;%d", &h, &v, &sv);
 
   handleHEAT(h);
   handleVENT(v);
   pSetpoint = sv; // Update setpoint if needed
 
   lastArtisanCommandTime = millis();
 }
 
 // PID hControls///
 // adjusting the heating power based on PID temperature control
 void handlePID() {
   if (pid.GetMode() == AUTOMATIC) {
     pInput = (double)temp;
     pid.Compute();
     handleHEAT((uint8_t)constrain((int)pOutput, 0, 100));
   } else {
     handleHEAT((uint8_t)constrain(manualHeatLevel, 0, 100));
   }
 }
 
 void setPIDMode(bool usePID) {
   if (usePID) {
     pid.SetMode(AUTOMATIC); // Enable PID
     isCooling = false;
     autoCoolingRearmOnHeat();
     D_println("PID mode set to AUTOMATIC");
   } else {
     pid.SetMode(MANUAL);         // Disable PID
     manualHeatLevel = 0;         // Set heat to 0% for safety
     handleHEAT(manualHeatLevel); // Apply the change immediately
     D_println("PID mode set to MANUAL");
   }
 }
 
 void notifyClient(const String &message) {
   // BLE notification (only if connected and subscribed)
   if (deviceConnected()) {
     // Limit to MTU-3 bytes
     size_t maxLen = bleMtu - 3;
     size_t msgLen = min(message.length(), maxLen);
 
     pTxCharacteristic->setValue((uint8_t*)message.c_str(), msgLen);
     // Send notification
     pTxCharacteristic->notify();
   }
 }
 
 void handleCHAN() {
   String message = "# Active channels set to 2100\r\n";
   D_println(message);
   notifyClient(message);
 }
 
 void handleREAD() {
   float displayTemp = (CorF == 'F') ? (temp * 1.8) + 32.0 : temp;
   String output = "0," + String(displayTemp, 1) + "," + String(displayTemp, 1) + "," + String(currentHeat) + "," + String(currentFan) + "\r\n";
   notifyClient(output);
 }
 
 void handleOT1(uint8_t value) {
   if (pid.GetMode() == AUTOMATIC) {
     setPIDMode(false); // Disable PID control
   }
 
   manualHeatLevel = constrain(value, 0, 100); // Set manual heat level
   handleHEAT(manualHeatLevel);                // Apply the new setting
   if (manualHeatLevel > 0) {
     isCooling = false;
   }
 }
 
 void handleHEAT(uint8_t value) {
   currentHeat = constrain(value, 0, 100);
   if (currentHeat > 0) {
     isCooling = false;
     // Re-arm auto-cooling because we've applied heat
     autoCoolingRearmOnHeat();
   }
 }
 
 void handleVENT(uint8_t value) {
   currentFan = constrain(value, 0, 100);
 }
 
 void handleDRUM(uint8_t value) {
   currentDrum = (value == 0) ? 0x00 : 0x02;
 }
 
 void handleCOOL(uint8_t value) {
   if (value > 0) {
     isCooling = true;
     currentHeat = 0; // Heat must be off for cooling
     manualHeatLevel = 0; // Same for PID heat
     setPIDMode(false);
     currentFan = constrain(value, 0, 100);
   } else {
     isCooling = false;
     currentFan = 0;
   }
 }
 
 void handleFILTER(uint8_t value) {
   if (value >= 1 && value <= 5) {
     filterValue = value;
     D_println("Filter set to " + String(value));
   }
 }
 
 void eStop() {
   D_println("Emergency Stop Activated! Heater OFF, Vent 100%");
   handleHEAT(0);   // Turn off heater
   handleCOOL(100); // Set cooling and vent to 100%
 }
 
 void parseAndExecuteCommands(String input, bool fromBLE) {
   input.trim();
   input.toUpperCase();
 
   if (input.startsWith("PID:")) {
     applyArtisanPID(input);
     return;
   }
 
   int split1 = input.indexOf(';');
   String command = "";
   String param = "";
   String subcommand = "";
 
   if (split1 >= 0) {
     command = input.substring(0, split1);
     String remainder = input.substring(split1 + 1);
     int split2 = remainder.indexOf(';');
 
     if (split2 >= 0) {
       subcommand = remainder.substring(0, split2);
       param = remainder.substring(split2 + 1);
     } else {
       param = remainder;
     }
   } else {
     command = input;
   }
 
   if (command == "PID") {
     if (param == "ON") {
       setPIDMode(true); // Enable PID control
     } else if (param == "OFF") {
       setPIDMode(false); // Disable PID control
     } else if (subcommand == "SV") {
       double newSetpoint = param.toDouble();
       if (newSetpoint > 0 && newSetpoint <= 220) { // Example range check
         pSetpoint = newSetpoint;
         D_println("New Setpoint: " + String(pSetpoint));
       }
     } else if (subcommand == "KP") {
       D_println("Setting KP to: " + String(param.toDouble()));
       Kp = param.toDouble();
       pid.SetTunings(Kp, Ki, Kd, pMode); // apply the pid params to running config
     } else if (subcommand == "KI") {
       D_println("Setting KI to: " + String(param.toDouble()));
       Ki = param.toDouble();
       pid.SetTunings(Kp, Ki, Kd, pMode); // apply the pid params to running config
     } else if (subcommand == "KD") {
       D_println("Setting KD to: " + String(param.toDouble()));
       Kd = param.toDouble();
       pid.SetTunings(Kp, Ki, Kd, pMode); // apply the pid params to running config
     } else if (subcommand == "PM") {
       D_println("Setting PMode to: " + param);
       if (param == "M") {
         pMode = P_ON_M;
         pid.SetTunings(Kp, Ki, Kd, pMode); // apply the pid params to running config
       } else {
         pMode = P_ON_E;
         pid.SetTunings(Kp, Ki, Kd, pMode); // apply the pid params to running config
       }
     }
   } else if (command == "OT1") {
     D_println("Setting OT1: " + param);
     handleOT1(param.toInt()); // Manual heater control (only in MANUAL mode)
   } else if (command == "READ") {
     handleREAD();
   } else if (command == "OT2") {
     D_println("Setting OT2: " + param);
     handleVENT(param.toInt()); // Set fan duty
   } else if (command == "OFF") {
     shutdown(); // Shut down system
   } else if (command == "ESTOP") {
     eStop(); // Emergency stop (heater = 0, vent = 100)
   } else if (command == "DRUM") {
     D_println("Setting Drum: " + param);
     handleDRUM(param.toInt()); // Start/stop the drum
   } else if (command == "FILTER") {
     D_println("Setting Filter: " + param);
     handleFILTER(param.toInt()); // Turn on/off filter fan
   } else if (command == "COOL") {
     D_println("Setting Cool: " + param);
     handleCOOL(param.toInt()); // Cool the beans
   } else if (command == "CHAN") {
     handleCHAN(); // Handle TC4 init message
   } else if (command == "UNITS") {
     if (split1 >= 0) CorF = input.charAt(split1 + 1); // Set temperature units
   }
 }
 
 // =============================================================================
 // BLE SETUP & CALLBACKS
 // =============================================================================
 bool deviceConnected() {
   return (pTxCharacteristic && pTxCharacteristic->getSubscribedCount() > 0) || webSocket.count() > 0;
 }
 
 class MyServerCallbacks : public NimBLEServerCallbacks {
   void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
     D_println("BLE Client Connected.");
   }
   void onDisconnect(NimBLEServer *pServer) {
     D_println("BLE Client Disconnected. Restarting advertising...");
     if (bleQueue) xQueueReset(bleQueue);
   }
 };
 
 class MyCallbacks : public NimBLECharacteristicCallbacks {
   void onWrite(NimBLECharacteristic *pCharacteristic) override {
     // SAFE: Minimal work in callback
     const std::string &rx = pCharacteristic->getValue();
     if (rx.empty()) return;
 
     BleMsg message;
     const size_t cap = sizeof(message.buf) - 1u;
     const size_t len = std::min(rx.size(), cap);
 
     message.len = static_cast<uint16_t>(len);
     memcpy(message.buf, rx.data(), len);
     message.buf[len] = '\0';
 
     if (xQueueSendToBack(bleQueue, &message, 0) != pdTRUE) {
       // Queue is full, drop oldest, enqueue newest (keeps most recent commands)
       BleMsg trash;
       (void)xQueueReceive(bleQueue, &trash, 0);
       (void)xQueueSendToBack(bleQueue, &message, 0);
     }
   }
 };
 
 class MyTxCallbacks : public NimBLECharacteristicCallbacks {
   void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue) override {
     if (subValue == 1) {
       D_println("BLE: device subscribed to notifications");
     } else if (subValue == 0) {
       D_println("BLE: device unsubscribed from notifications");
     }
     D_printf("BLE: Subscription value: %u, Total subscribers: %u\n", subValue, pCharacteristic->getSubscribedCount());
   }
 };
 
 void initBLE() {
   NimBLEDevice::init(boardID_BLE.c_str());
   NimBLEDevice::setPower(ESP_PWR_LVL_N0);
   NimBLEDevice::setMTU(bleMtu);
   NimBLEDevice::setSecurityAuth(false, false, false);
 
   pServer = NimBLEDevice::createServer();
   pServer->setCallbacks(new MyServerCallbacks());
   pServer->advertiseOnDisconnect(true); // auto-adv when peer drops
 
   NimBLEService *svc = pServer->createService(SERVICE_UUID);
   pTxCharacteristic = svc->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
   pTxCharacteristic->setCallbacks(new MyTxCallbacks()); // Add subscription callback
 
   auto rx = svc->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
   rx->setCallbacks(new MyCallbacks());
 
   // Create a queue for the BLE messages with maximum depth
   bleQueue = xQueueCreateStatic(BLE_Q_LEN, sizeof(BleMsg), bleQStorage, &bleQCtrl);
   configASSERT(bleQueue);
 
   svc->start();
 
   // Standard advertising parameters for maximum compatibility
   NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
   adv->addServiceUUID(SERVICE_UUID); // critical for Android discovery
   adv->setScanResponse(true);
   adv->setAppearance(0x00);
   adv->setMinInterval(0x20); // 32 slots = 20ms
   adv->setMaxInterval(0x40); // 64 slots = 40ms
   adv->start();
 
   D_println("BLE Advertising started with standard parameters.");
 }
 
 static void processBleQueueBurst() {
   const UBaseType_t MAX_PER_BURST = BLE_Q_LEN;
   constexpr unsigned YIELD_EVERY = 4;
 
   BleMsg m;
   for (UBaseType_t i = 0; i < MAX_PER_BURST; ++i) {
     if (xQueueReceive(bleQueue, &m, 0) != pdTRUE) break;
     parseAndExecuteCommands(String(m.buf, m.len), true);
     if ((i+1) % YIELD_EVERY == 0) taskYIELD();
   }
 }
 
 // =============================================================================
 // WIFI & WEB SERVER SETUP
 // =============================================================================
 void sendArtisanCompatibleData(AsyncWebSocketClient *client, long commandId);
 
 void setupAP() {
   WiFi.mode(WIFI_AP);
   WiFi.softAP(wifiAPSSID, wifiAPPass);
   D_print("AP Mode Started. SSID: ");
   D_println(wifiAPSSID);
 }
 
 void connectToWifi() {
   preferences.begin("wifi", true);
   String ssid = preferences.getString("ssid", "");
   String pass = preferences.getString("pass", "");
   preferences.end();
 
   if (ssid == "") {
     setupAP();
     return;
   }
 
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid.c_str(), pass.c_str());
   D_print("Connecting to WiFi ");
   int attempts = 0;
   while (WiFi.status() != WL_CONNECTED && attempts < 20) {
     delay(500);
     D_print(".");
     attempts++;
   }
 
   if (WiFi.status() != WL_CONNECTED) {
     D_println("\nFailed to connect. Starting AP mode.");
     setupAP();
   } else {
     D_println("\nConnected to WiFi.");
     D_print("IP: ");
     D_println(WiFi.localIP());
 
     if (!MDNS.begin("cubean")) {
       D_println("Error setting up MDNS responder");
     }
   }
 }
 
 // -----------------------------------------------------------------------------
 // UI
 // -----------------------------------------------------------------------------
 
 // Single stylesheet for ALL pages
 static const uint8_t APP_CSS[] PROGMEM = R"CSS(
 :root{--bg:#0b0f14;--panel:#0f172a;--muted:#94a3b8;--text:#e5e7eb;--text-dim:#cbd5e1;--chip:#0b1220;--border:#1f2937;--accent:#3b82f6;--good:#10b981;--warn:#f59e0b;--bad:#ef4444}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial;line-height:1.45}
 .container{max-width:1100px;margin:24px auto;background:var(--panel);padding:24px;border-radius:14px;border:1px solid var(--border);box-shadow:0 10px 30px rgba(0,0,0,.35),inset 0 1px 0 rgba(255,255,255,.03)}
 h1{margin:0 0 8px;text-align:center;letter-spacing:.2px;font-weight:700}
 h2{margin:18px 0 10px;font-size:1.05rem;color:var(--text-dim);border-bottom:1px solid var(--border);padding-bottom:8px}
 .sub{color:var(--muted);text-align:center}
 a{color:#93c5fd;text-decoration:none} a:hover{text-decoration:underline}
 ul{padding-left:18px;margin:8px 0 0} li{margin:6px 0}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}
 .card{background:linear-gradient(180deg,rgba(255,255,255,.03),rgba(255,255,255,0));border:1px solid var(--border);padding:16px;border-radius:12px}
 label{display:block;margin:8px 0 6px;color:var(--muted)}
 input[type=text],input[type=password],input[type=file]{width:100%;background:var(--chip);border:1px solid var(--border);padding:10px;border-radius:10px;color:var(--text)}
 .btn{padding:10px 14px;border-radius:10px;border:1px solid var(--border);background:var(--accent);color:#fff;font-weight:600;cursor:pointer}
 .badge{display:inline-block;padding:2px 8px;border-radius:999px;background:var(--chip);border:1px solid var(--border);color:var(--text-dim);font-size:.85rem}
 .bar{height:10px;background:var(--chip);border:1px solid var(--border);border-radius:999px;margin-top:14px;overflow:hidden}
 .bar>div{height:100%;width:0%;background:linear-gradient(90deg,rgba(59,130,246,.75),rgba(59,130,246,1))}
 .log{margin-top:12px;color:var(--text-dim);font-family:ui-monospace,Menlo,Consolas,monospace;white-space:pre-wrap}
 .ok{color:var(--good)} .err{color:var(--bad)} .warn{color:var(--warn)}
 @media (max-width:700px){.container{margin:12px;padding:16px}}
 
 /* OTA (homepage) */
 .drop{margin-top:10px;border:2px dashed rgba(148,163,184,.4);border-radius:12px;background:rgba(11,18,32,.35);padding:26px;text-align:center;transition:all .15s ease}
 .drop:hover{border-color:#93c5fd;background:rgba(59,130,246,.05);cursor:pointer}
 .drop.dragover{border-color:#3b82f6;background:rgba(59,130,246,.12);box-shadow:0 0 0 3px rgba(59,130,246,.15) inset}
 .hint{color:var(--muted);font-size:.95rem;margin-top:6px}
 #fileInput{display:none}
 )CSS";
 
 static const char homepage_html[] PROGMEM = R"(
 <html>
 <head>
   <meta charset="utf-8">
   <meta name="viewport" content="width=device-width,initial-scale=1">
   <title>Cubean Homepage</title>
   <link rel="stylesheet" href="/app.css">
 </head>
 <body>
   <div class="container">
     <h1>Cubean Setup</h1>
     <div class="sub">Configure Wi-Fi, open monitors, or update firmware</div>
 
     <div class="grid">
       <!-- WiFi setup -->
       <div class="card">
         <h2>Wi-Fi Setup</h2>
         <form action="/save" method="post">
           <label for="ssid">SSID</label>
           <input type="text" id="ssid" name="ssid" placeholder="Your Wi-Fi name">
           <label for="pass">Password</label>
           <input type="password" id="pass" name="pass" placeholder="Wi-Fi password">
           <div style="margin-top:12px"><button class="btn" type="submit">Save & Restart</button></div>
         </form>
       </div>
 
       <!-- Links -->
       <div class="card">
         <h2>Pages</h2>
         <ul>
           <li><a href="/webserial">WebSerial</a> — WebSerial debug</li>
           <li><a href="/">Home</a> — quick actions & OTA</li>
         </ul>
       </div>
     </div>
 
     <!-- Drag & Drop OTA -->
     <div class="card" style="margin-top:14px">
       <h2>OTA Firmware Update (Current: <span id="fwv">--</span>)</h2>
       <div id="drop" class="drop">
         <div style="font-weight:600;margin-bottom:4px">Drag & drop your <code>.bin</code> here</div>
         <div class="hint">or click to choose a file</div>
       </div>
       <input id="fileInput" type="file" accept=".bin">
       <div class="bar"><div id="bar"></div></div>
       <div id="log" class="log"></div>
     </div>
   </div>
 
   <script>
     const drop = document.getElementById('drop');
     const fileInput = document.getElementById('fileInput');
     const bar = document.getElementById('bar');
     const log = document.getElementById('log');
 
     function logLine(m,c){const e=document.createElement('div');if(c)e.className=c;e.textContent=m;log.appendChild(e);log.scrollTop=log.scrollHeight;}
 
     function uploadFile(file){
       if(!file){ return; }
       const name = (file.name || '').toLowerCase();
       if (!name.endsWith('.bin')) {
         logLine('Please select a .bin firmware','err');
         return;
       }
       drop.classList.remove('dragover');
       drop.style.pointerEvents='none';
 
       log.innerHTML='';
       bar.style.width='0%';
       logLine(`Uploading ${file.name} (${(file.size/1048576).toFixed(2)} MB)...`);
 
       const xhr = new XMLHttpRequest();
       xhr.open('POST','/update',true);
       xhr.timeout = 300000; // 5 minute timeout
       xhr.setRequestHeader('X-File-Name',encodeURIComponent(file.name));
 
       let uploadCompleteShown = false;
       xhr.upload.onprogress = function(e){
         if(e.lengthComputable){
           const pct = Math.round((e.loaded/e.total)*100);
           bar.style.width = pct + '%';
           if(pct === 100 && !uploadCompleteShown) {
             uploadCompleteShown = true;
             logLine('Upload complete, processing firmware...', 'ok');
           }
         }
       };
 
       xhr.onload = function(){
         if(xhr.status === 200){
           const responseText = xhr.responseText || '';
           const ok = responseText.includes('Update OK') || responseText.includes('OK');
 
           if(ok){
             logLine('Firmware update successful! Device is rebooting...','ok');
             logLine('Page will refresh in 10 seconds (waiting for device restart)','ok');
             setTimeout(()=>{
               logLine('Refreshing page...','ok');
               location.href='/';
             }, 10000);
           }
           else {
             logLine('Update failed: ' + responseText,'err');
             logLine('Re-enabling upload area','err');
             drop.style.pointerEvents='auto';
           }
         } else {
           logLine('HTTP '+xhr.status+' – '+xhr.responseText, 'err');
           drop.style.pointerEvents='auto';
           bar.style.width='0%';
         }
       };
 
       xhr.ontimeout = function(){
         logLine('Upload timeout - device may have restarted successfully','warn');
         logLine('Please wait and refresh page manually if needed','warn');
         setTimeout(()=>location.href='/',10000);
       };
 
       xhr.onerror = function(){
         logLine('Network error during upload','err');
         drop.style.pointerEvents='auto';
         bar.style.width='0%';
       };
 
       const form = new FormData();
       form.append('firmware', file, file.name);
       xhr.send(form);
     }
 
     // Click to open file picker
     drop.addEventListener('click', ()=> fileInput.click());
     fileInput.addEventListener('change', ()=> uploadFile(fileInput.files[0]));
 
     // Drag & drop handlers
     ['dragenter','dragover'].forEach(ev=>{
       drop.addEventListener(ev, e=>{ e.preventDefault(); e.stopPropagation(); drop.classList.add('dragover'); });
     });
     ['dragleave','drop'].forEach(ev=>{
       drop.addEventListener(ev, e=>{ e.preventDefault(); e.stopPropagation(); drop.classList.remove('dragover'); });
     });
     drop.addEventListener('drop', e=>{
       const files = e.dataTransfer.files;
       if(files && files.length) uploadFile(files[0]);
     });
 
     // Prevent the browser from opening the file if dropped outside the box
     ['dragover','drop'].forEach(ev=>{
       window.addEventListener(ev, e=>{ e.preventDefault(); });
     });
   </script>
   <script src="/fw.js"></script>
 </body>
 </html>
 )";
 
 void handleRoot(AsyncWebServerRequest *req) {
   req->send(200, "text/html", homepage_html);
 }
 
 void handleCss(AsyncWebServerRequest *req) {
   size_t css_len = sizeof(APP_CSS);
   AsyncWebServerResponse *res = req->beginResponse(200, "text/css", APP_CSS, css_len);
   res->addHeader("Cache-Control", "public, max-age=31536000, immutable");
   req->send(res);
 }
 
 void handleSave(AsyncWebServerRequest *req) {
   const String ssid = req->hasParam("ssid", true) ? req->getParam("ssid", true)->value() : "";
   const String pass = req->hasParam("pass", true) ? req->getParam("pass", true)->value() : "";
 
   preferences.begin("wifi", false);
   preferences.putString("ssid", ssid);
   preferences.putString("pass", pass);
   preferences.end();
 
   req->send(200, "text/html", "<h1>Settings Saved. Restarting...</h1>");
   delay(1000);
   ESP.restart();
 }
 
 void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
   if (type == WS_EVT_CONNECT) {
     D_printf("[WS] #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
     return;
   }
 
   if (type == WS_EVT_DISCONNECT) {
     D_printf("[WS] #%u disconnected\n", client->id());
     return;
   }
 
   if (type == WS_EVT_DATA) {
     auto *info = (AwsFrameInfo *)arg;
     if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;
 
     if (len > 200) {
       D_println("WS: payload too large");
       return;
     }
 
     D_printf("Received message: %.*s\n", (int)len, (const char *)data);
 
     // Try JSON first (for { "command":"getData", "id": N })
     JsonDocument doc;
     DeserializationError err = deserializeJson(doc, (const char *)data, len);
 
     if (err) {
       // Fallback: treat as plain-text command (create String once, bounded)
       String msg((const char *)data, len);
       D_println("Received message: " + msg);
       parseAndExecuteCommands(msg, false);
     }
 
     // handle getData
     const char *command = doc["command"];
     if (command && strcmp(command, "getData") == 0) {
       sendArtisanCompatibleData(client, doc["id"] | 0L);
       return;
     }
 
     // Get BurnerVal from Artisan over Websocket
     if (!doc["BurnerVal"].isNull()) {
       unsigned char val = doc["BurnerVal"].as<unsigned char>();
       D_printf("BurnerVal: %d\n", val);
       handleOT1(val);
     }
     if (!doc["FanVal"].isNull()) {
       unsigned char val = doc["FanVal"].as<unsigned char>();
       D_printf("FanVal: %d\n", val);
       handleVENT(val);
     }
     if (!doc["Drum"].isNull()) {
       unsigned char val = doc["Drum"].as<unsigned char>();
       D_printf("Drum: %d\n", val);
       handleDRUM(val);
     }
     if (!doc["Cooling"].isNull()) {
       unsigned char val = doc["Cooling"].as<unsigned char>();
       D_printf("Cooling: %d\n", val);
       handleCOOL(val);
     }
     if (!doc["Cool"].isNull()) {
       unsigned char val = doc["Cool"].as<unsigned char>();
       D_printf("Cooling: %d\n", val);
       handleCOOL(val);
     }
   }
 }
 
 void sendArtisanCompatibleData(AsyncWebSocketClient *client, long commandId) {
   JsonDocument doc;
   doc["id"] = commandId;
   JsonObject data = doc["data"].to<JsonObject>();
 
   float displayTemp = (CorF == 'F') ? (temp * 1.8) + 32.0 : temp;
   float roundedTemp = round(displayTemp * 10) / 10.0;
 
   data["ET"] = roundedTemp;
   data["BT"] = roundedTemp;
   data["BurnerVal"] = currentHeat;
   data["FanVal"] = currentFan;
   data["Drum"] = currentDrum;
   data["Cool"] = isCooling ? 100 : 0;
 
   char buf[256];
   size_t len = serializeJson(doc, buf, sizeof(buf)); // bounded
   if (!len) {
     String out;
     out.reserve(measureJson(doc) + 1);
     serializeJson(doc, out);
     if (client && client->status() == WS_CONNECTED) client->text(out);
     return;
   }
   if (client && client->status() == WS_CONNECTED) client->text(buf, len);
 }
 
 void handleFirmwareJS(AsyncWebServerRequest *req) {
   auto *res = req->beginResponseStream("application/javascript");
   res->addHeader("Cache-Control", "no-store");
   res->printf("const FW_VER=\"%s\";"
               "var e=document.getElementById('fwv');"
               "if(e) e.textContent=FW_VER;", firmWareVersion);
   req->send(res);
 }
 
 void handleUpdateComplete(AsyncWebServerRequest *req) { // finished
   // Always reset OTA flag first, regardless of success/failure
   otaActive = false;
 
   const bool ok = !Update.hasError();
   if (ok) {
     D_println("OTA Update completed successfully - scheduling restart");
     req->send(200, "text/plain", "Update OK, rebooting...");
 
     // Schedule restart in 5 seconds (non-blocking)
     restartPending = true;
     restartTime = millis() + 5000;
     D_println("Restart scheduled for 5 seconds from now");
   } else {
     D_println("OTA Update failed");
     Update.printError(Serial);
     req->send(500, "text/plain", "Update FAILED - check WebSerial for details");
   }
 }
 
 void handleUpdateUpload(AsyncWebServerRequest *req, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
   if (index == 0) {
     otaActive = true;
     otaStartTime = millis();
     D_printf("OTA start: %s\n", filename.c_str());
     // Unknown total size: the core will page it in
     if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
       Update.printError(Serial); // Print& required — use Serial
     }
   }
 
   if (len) {
     if (Update.write(data, len) != len) {
       Update.printError(Serial);
     }
     yield(); // keep WiFi alive
   }
 
   if (final) {
     if (!Update.end(true)) { // true = set boot partition
       Update.printError(Serial);
       otaActive = false;
     } else {
       D_printf("OTA success: %u bytes\n", (unsigned)(index + len));
     }
   }
 }
 
 void setupWebServer() {
   DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) { handleRoot(req); });
   server.on("/app.css", HTTP_GET, [](AsyncWebServerRequest *req) { handleCss(req); });
   server.on("/save", HTTP_POST, [](AsyncWebServerRequest *req) { handleSave(req); });
 
   // Firmware version
   server.on("/fw.js", HTTP_GET, [](AsyncWebServerRequest *req) { handleFirmwareJS(req); });
   // OTA upload endpoint: streams file to OTA partition
   server.on(
       "/update", HTTP_POST, [](AsyncWebServerRequest *req) { handleUpdateComplete(req); },
       [](AsyncWebServerRequest *req, const String &fn, size_t idx, uint8_t *data, size_t len, bool final) {
         handleUpdateUpload(req, fn, idx, data, len, final);
       },
       nullptr);
 
   // --- WebSockets ---
   webSocket.onEvent(webSocketEvent);
   server.addHandler(&webSocket);
 
#if SERIAL_DEBUG == 1
   // --- WebSerial on the same port ---
   // WebSerial.setAuthentication("admin", "changeme");   // strongly recommended
   WebSerial.onMessage([](uint8_t *data, size_t len) {
     static char buffer[512]; // Static buffer, no heap allocation
     size_t copyLen = min(len, sizeof(buffer) - 1);
     memcpy(buffer, data, copyLen);
     buffer[copyLen] = '\0'; // Ensure null termination
 
     if (strcmp(buffer, "status") == 0) {
       WebSerial.printf("heap: free=%u (%.1f KB)  min=%u  maxblk=%u  psram_free=%u\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024.0,
                        ESP.getMinFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());
     } else {
       WebSerial.printf("rx: %s\n", buffer);
     }
   });
   WebSerial.begin(&server); // mounts at /webserial
   g_webSerialInit = true; 
 #endif

   server.begin();
 }
 
 // =============================================================================
 // MAIN SETUP & LOOP
 // =============================================================================
 
 void setup() {
   Serial.begin(115200);
   Serial.printf("Starting Firmware: %s\n", firmWareVersion);
 
   FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
   FastLED.setBrightness(255); // Set maximum brightness
   setRGBColor(LED_RED);
 
   roaster_uart_init();
   panel_uart_init();
 
   initBLE();
   connectToWifi();
   setupWebServer();
 
   pid.SetMode(MANUAL); // Start in manual mode
   pid.SetOutputLimits(0, 100);
   pid.SetSampleTime(1000);
 
   shutdown();
   delay(1000);
   setRGBColor(CRGB::Black);
 }
 
 void loop() {
   // Check for OTA timeout and reset if stuck
   if (otaActive) {
     if (millis() - otaStartTime > OTA_TIMEOUT_MS) {
       D_println("ERROR: OTA timeout - resetting OTA flag");
       otaActive = false;
       Update.abort();
     } else {
       delay(100);
       return; // skip everything else
     }
   }
   // Check for pending restart after OTA update
   if (restartPending && millis() >= restartTime) {
     D_println("Executing scheduled restart after OTA update");
     ESP.restart();
   }
 
   static unsigned long lastSendTime = 0;
   const unsigned long sendInterval = 200;
 
   getPanelMessage();
   yield();
   getRoasterMessage();
   yield();
 
   if (millis() - lastSendTime > sendInterval) {
     lastSendTime = millis();
     handlePID();
     sendRoasterMessage();
     yield();
   }
 
   if (Serial.available()) {
     String line = Serial.readStringUntil('\n');
     parseAndExecuteCommands(line, false);
     yield();
   }
 
   processBleQueueBurst();
 
   handleLED();
   delay(20);
 }
 