/*
 * WT32-ETH01 (ESP32 + LAN8720) — Modbus TCP Server over Ethernet
 *
 * Board: WT32-ETH01 V1.4
 * PHY:   LAN8720A (RMII)
 * Library: eModbus (ModbusServerETH)
 *
 * Ethernet Pin Mapping:
 *   MDC       -> GPIO 23
 *   MDIO      -> GPIO 18
 *   CLK       -> GPIO 0
 *   PHY Power -> GPIO 16
 *   PHY Addr  -> 1
 *
 * Modbus Map:
 *   Function : Read Holding Registers (0x03)
 *   Registers: 100 to 107 (8 sensors)
 *   Port     : 502
 *   Unit ID  : 1
 *
 * IMPORTANT: ETH #defines MUST appear before #include <ETH.h>
 */

// ── WT32-ETH01 PHY configuration ────────────────────────────────────
#define ETH_PHY_TYPE    ETH_PHY_LAN8720
#define ETH_PHY_ADDR    1
#define ETH_PHY_MDC     23
#define ETH_PHY_MDIO    18
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN   16
// ─────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <ETH.h>
#include <ModbusServerETH.h>   // eModbus – Ethernet TCP server

// ── Modbus Server & Register Map ─────────────────────────────────────
ModbusServerEthernet MBserver;

// Sensor values and Switch Pins (8 sensors)
uint16_t sensorValues[8] = {0};
const int switchPins[8] = {2, 4, 12, 14, 15, 36, 39, 35};

const uint8_t  MODBUS_SERVER_ID  = 1;
const uint16_t SENSOR_REG_START  = 100;  // Holding register start address
const uint16_t SENSOR_COUNT      = 8;
const uint16_t MODBUS_PORT       = 502;

// ── State ─────────────────────────────────────────────────────────────
static bool eth_connected = false;
static bool eth_has_ip    = false;
static bool mb_started    = false;

// ── Modbus FC03 handler: Read Holding Registers ───────────────────────
ModbusMessage FC03handler(ModbusMessage request) {
  ModbusMessage response;

  uint16_t startAddr = 0;
  uint16_t numRegs   = 0;
  request.get(2, startAddr);
  request.get(4, numRegs);

  Serial.printf("[Modbus] FC03 read addr=%d count=%d\n", startAddr, numRegs);

  // Validate range
  if (startAddr < SENSOR_REG_START ||
      (startAddr + numRegs) > (SENSOR_REG_START + SENSOR_COUNT)) {
    // Out of range — return Illegal Data Address exception
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  // Build response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(numRegs * 2));
  for (uint16_t i = 0; i < numRegs; i++) {
    uint16_t idx = startAddr - SENSOR_REG_START + i;
    response.add(sensorValues[idx]);
  }

  return response;
}

// ── Ethernet event handler ────────────────────────────────────────────
void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Ethernet PHY started");
      ETH.setHostname("wt32-modbus");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Link UP");
      eth_connected = true;
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_has_ip = true;
      Serial.println("[ETH] IP obtained via DHCP");
      Serial.print  ("  IPv4  : "); Serial.println(ETH.localIP());
      Serial.print  ("  GW    : "); Serial.println(ETH.gatewayIP());
      Serial.print  ("  Speed : "); Serial.print(ETH.linkSpeed()); Serial.println(" Mbps");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Link DOWN");
      eth_connected = false;
      eth_has_ip    = false;
      mb_started    = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Stopped");
      eth_connected = false;
      eth_has_ip    = false;
      mb_started    = false;
      break;

    default:
      break;
  }
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=========================================");
  Serial.println("  WT32-ETH01  —  Modbus TCP Server v1.0 ");
  Serial.println("=========================================");
  Serial.println();

  // Register Modbus FC03 handler
  MBserver.registerWorker(MODBUS_SERVER_ID, READ_HOLD_REGISTER, &FC03handler);

  // Start Ethernet
  WiFi.onEvent(onEthEvent);
  ETH.begin();

  // Configure switch pins as INPUT
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(switchPins[i], INPUT);
  }

  Serial.println("[ETH] Initialising LAN8720 PHY ...");
  Serial.println("[ETH] Waiting for cable & DHCP ...");
}

// ── Loop ──────────────────────────────────────────────────────────────
void loop() {
  // Start Modbus TCP server once Ethernet is up (only once)
  if (eth_has_ip && !mb_started) {
    MBserver.start(MODBUS_PORT, 5, 20000);  // port 502, max 5 clients, 20s timeout
    mb_started = true;
    Serial.print("[Modbus] Server started on ");
    Serial.print(ETH.localIP());
    Serial.printf(":%d  (registers %d-%d)\n",
                  MODBUS_PORT,
                  SENSOR_REG_START,
                  SENSOR_REG_START + SENSOR_COUNT - 1);
  }

  // Read real switch data every second (or faster if you prefer)
  static unsigned long last_sim_ms = 0;
  if (millis() - last_sim_ms > 1000) {
    last_sim_ms = millis();
    for (int i = 0; i < SENSOR_COUNT; i++) {
      sensorValues[i] = digitalRead(switchPins[i]);  // Read actual switch states
    }
    Serial.print("[Switches] ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.printf("S%d=%d ", i + 1, sensorValues[i]);
    }
    Serial.println();
  }
}
