/*
 * ETH01 EVO (ESP32-C3) VoIP / SIP Intercom Code
 * 
 * Features:
 * 1. Low-Latency Full-Duplex Audio Engine (INMP441 Mic + MAX98357A Amp).
 * 2. Speaker Output Gain Control & Anti-Choppy UDP Jitter Drain.
 * 3. SIP Auto-Answer Mode with explicit "Ringing" (180 Ringing) and "Auto-Answer" (200 OK) notifications.
 * 4. DM9051 SPI Ethernet Driver for ETH01 EVO Board.
 * 
 * Pin Configuration:
 *   DM9051 Ethernet (SPI): CS=9, IRQ=8, RST=6, SCK=7, MISO=3, MOSI=10
 *   I2S Audio (ESP32-C3):
 *     Mic: BCLK=18, WS=19, SD=1
 *     Amp: BCLK=5,  LRC=2, DIN=4
 */

// ── 1. SIP Credentials & Audio Tuning Settings ──────────────────────
const char* SIP_SERVER_IP   = "192.168.0.113"; // PBX / Asterisk Server IP
const int   SIP_SERVER_PORT = 5060;           // PBX Server SIP Port
const char* SIP_USER        = "100";           // Extension / Username (100/101/200)
const char* SIP_PASSWORD    = "1234";          // Extension Password

#define ENABLE_SIP_REGISTER  true             // Set to true for Registration, false for Direct IP peer

// Speaker & Microphone Audio Gain Control
float SPEAKER_GAIN = 1.0f;                    // Adjustable Speaker Gain (0.5 = quieter/cleaner, 1.0 = normal, 2.0 = louder)
int   MIC_GAIN     = 8;                       // Digital Mic Boost Multiplier (4, 8, 16)
// ─────────────────────────────────────────────────────────────────────

// ── 2. DM9051 Ethernet PHY & SPI Configuration ──────────────────────
#define ETH_PHY_TYPE        ETH_PHY_DM9051
#define ETH_PHY_ADDR        1

#define ETH_PHY_CS          9
#define ETH_PHY_IRQ         8
#define ETH_PHY_RST         6

#define ETH_PHY_SPI_HOST    SPI2_HOST
#define ETH_PHY_SPI_SCK     7
#define ETH_PHY_SPI_MISO    3
#define ETH_PHY_SPI_MOSI    10
// ─────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <MD5Builder.h>
#include "driver/i2s.h"

#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/usb_serial_jtag_reg.h>
#include <soc/gpio_periph.h>
#include <driver/gpio.h>

// --- I2S Configuration for ESP32-C3 Single Port ---
#define I2S_PORT_NUM     I2S_NUM_0

// Microphone (RX) Pins
#define I2S_RX_BCLK_PIN  18
#define I2S_RX_LRC_PIN   19
#define I2S_RX_DOUT_PIN  1

// Speaker (TX) Pins
#define I2S_TX_BCLK_PIN  5
#define I2S_TX_LRC_PIN   2
#define I2S_TX_DIN_PIN   4

#define SAMPLE_RATE      8000 // Standard for G.711 VoIP

WiFiUDP udpRtpRx;
WiFiUDP udpSip;

const int SIP_PORT = 5060;
const int RTP_PORT_LOCAL = 7078;
int remote_rtp_port = 8000;
IPAddress remote_ip;

bool ethernet_connected = false;
bool in_call = false;
bool sip_registered = false;

String call_id = "";
String reg_call_id = "";
String reg_from_tag = "";
uint32_t reg_cseq = 1;
unsigned long last_reg_time = 0;

// --- G.711 u-law compression lookup tables ---
static const int16_t ulaw_decode_table[256] = {
   -32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956,
   -23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764,
   -15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412,
   -11900,-11388,-10876,-10364, -9852, -9340, -8828, -8316,
    -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140,
    -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092,
    -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004,
    -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
    -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436,
    -1372, -1308, -1244, -1180, -1116, -1052,  -988,  -924,
     -876,  -844,  -812,  -780,  -748,  -716,  -684,  -652,
     -620,  -588,  -556,  -524,  -492,  -460,  -428,  -396,
     -372,  -356,  -340,  -324,  -308,  -292,  -276,  -260,
     -244,  -228,  -212,  -196,  -180,  -164,  -148,  -132,
     -120,  -112,  -104,   -96,   -88,   -80,   -72,   -64,
      -56,   -48,   -40,   -32,   -24,   -16,    -8,     0,
    32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956,
    23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764,
    15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412,
    11900, 11388, 10876, 10364,  9852,  9340,  8828,  8316,
     7932,  7676,  7420,  7164,  6908,  6652,  6396,  6140,
     5884,  5628,  5372,  5116,  4860,  4604,  4348,  4092,
     3900,  3772,  3644,  3516,  3388,  3260,  3132,  3004,
     2876,  2748,  2620,  2492,  2364,  2236,  2108,  1980,
     1884,  1820,  1756,  1692,  1628,  1564,  1500,  1436,
      1372,  1308,  1244,  1180,  1116,  1052,   988,   924,
      876,   844,   812,   780,   748,   716,   684,   652,
      620,   588,   556,   524,   492,   460,   428,   396,
      372,   356,   340,   324,   308,   292,   276,   260,
      244,   228,   212,   196,   180,   164,   148,   132,
      120,   112,   104,    96,    88,    80,    72,    64,
       56,    48,    40,    32,    24,    16,     8,     0
};

uint8_t pcm_to_ulaw(int16_t pcm_val) {
    int16_t sign = (pcm_val >> 8) & 0x80;
    if (sign != 0) pcm_val = -pcm_val;
    if (pcm_val > 32635) pcm_val = 32635;
    pcm_val += 132;
    int exponent = 7;
    int mask = 0x4000;
    while ((pcm_val & mask) == 0 && exponent > 0) {
        exponent--;
        mask >>= 1;
    }
    int mantissa = (pcm_val >> (exponent + 3)) & 0x0f;
    uint8_t ulawbyte = ~(sign | (exponent << 4) | mantissa);
    return ulawbyte;
}

String getMD5(String input) {
  MD5Builder md5;
  md5.begin();
  md5.add(input);
  md5.calculate();
  return md5.toString();
}

String extractString(String source, String start, String end) {
  int startIndex = source.indexOf(start);
  if (startIndex == -1) return "";
  startIndex += start.length();
  int endIndex = source.indexOf(end, startIndex);
  if (endIndex == -1) return source.substring(startIndex);
  return source.substring(startIndex, endIndex);
}

String extractParam(String source, String paramKey) {
  int idx = source.indexOf(paramKey);
  if (idx == -1) return "";
  idx += paramKey.length();
  
  if (source.charAt(idx) == '"') {
    idx++;
    int endIdx = source.indexOf('"', idx);
    if (endIdx != -1) return source.substring(idx, endIdx);
  } else {
    int endIdx = source.indexOf(',', idx);
    if (endIdx == -1) endIdx = source.indexOf("\r\n", idx);
    if (endIdx == -1) endIdx = source.indexOf(' ', idx);
    if (endIdx == -1) endIdx = source.length();
    return source.substring(idx, endIdx);
  }
  return "";
}

void setup_i2s() {
  // Disconnect USB PHY from GPIO 18 and GPIO 19 so they function as standard GPIOs
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_BCLK_PIN], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_LRC_PIN],  PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_DOUT_PIN], PIN_FUNC_GPIO);

  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_BCLK_PIN], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_LRC_PIN],  PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_DIN_PIN],  PIN_FUNC_GPIO);

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_TX_BCLK_PIN,   // Speaker BCLK
      .ws_io_num = I2S_TX_LRC_PIN,     // Speaker LRC
      .data_out_num = I2S_TX_DIN_PIN,  // Speaker DIN
      .data_in_num = I2S_RX_DOUT_PIN   // Mic SD (Data In)
  };

  i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT_NUM, &pin_config);

  // Route active master clock lines to Mic BCLK & LRC pins via ESP32-C3 GPIO Matrix
  gpio_set_direction((gpio_num_t)I2S_RX_BCLK_PIN, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction((gpio_num_t)I2S_RX_LRC_PIN, GPIO_MODE_INPUT_OUTPUT);
  esp_rom_gpio_connect_out_signal(I2S_RX_BCLK_PIN, I2SO_BCK_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(I2S_RX_LRC_PIN, I2SO_WS_OUT_IDX, false, false);

  // Connect Mic BCLK & WS pins back to the I2S RX receiver clock inputs
  esp_rom_gpio_connect_in_signal(I2S_RX_BCLK_PIN, I2SI_BCK_IN_IDX, false);
  esp_rom_gpio_connect_in_signal(I2S_RX_LRC_PIN, I2SI_WS_IN_IDX, false);

  // Route Mic Data In pin explicitly to I2S RX Data Input signal
  gpio_set_direction((gpio_num_t)I2S_RX_DOUT_PIN, GPIO_MODE_INPUT);
  esp_rom_gpio_connect_in_signal(I2S_RX_DOUT_PIN, I2SI_SD_IN_IDX, false);

  i2s_start(I2S_PORT_NUM);
}

void sendSipRegisterInitial() {
  if (!ethernet_connected || !ENABLE_SIP_REGISTER) return;

  String local_ip = ETH.localIP().toString();
  if (reg_call_id == "") {
    reg_call_id = "esp32-reg-" + String(random(10000, 99999)) + "@" + local_ip;
  }
  if (reg_from_tag == "") {
    reg_from_tag = String(random(100000, 999999));
  }

  String server_uri = "sip:" + String(SIP_SERVER_IP);

  String reg = "REGISTER " + server_uri + " SIP/2.0\r\n";
  reg += "Via: SIP/2.0/UDP " + local_ip + ":" + String(SIP_PORT) + ";rport;branch=z9hG4bK-reg-" + String(reg_cseq) + "\r\n";
  reg += "From: <sip:" + String(SIP_USER) + "@" + String(SIP_SERVER_IP) + ">;tag=" + reg_from_tag + "\r\n";
  reg += "To: <sip:" + String(SIP_USER) + "@" + String(SIP_SERVER_IP) + ">\r\n";
  reg += "Call-ID: " + reg_call_id + "\r\n";
  reg += "CSeq: " + String(reg_cseq) + " REGISTER\r\n";
  reg += "Contact: <sip:" + String(SIP_USER) + "@" + local_ip + ":" + String(SIP_PORT) + ">\r\n";
  reg += "Max-Forwards: 70\r\n";
  reg += "Expires: 120\r\n";
  reg += "User-Agent: ESP32-ETH01-EVO\r\n";
  reg += "Content-Length: 0\r\n\r\n";

  udpSip.beginPacket(SIP_SERVER_IP, SIP_SERVER_PORT);
  udpSip.print(reg);
  udpSip.endPacket();

  Serial.println("[SIP] >>> Sent Initial REGISTER request to " + String(SIP_SERVER_IP));
}

void sendSipRegisterAuth(String realm, String nonce, String opaque, String qop) {
  if (!ethernet_connected || !ENABLE_SIP_REGISTER) return;

  reg_cseq++;
  String local_ip = ETH.localIP().toString();
  String server_uri = "sip:" + String(SIP_SERVER_IP);
  String cnonce = "0a4f113b";
  String nc = "00000001";

  // MD5 Digest Authentication Algorithm
  String ha1 = getMD5(String(SIP_USER) + ":" + realm + ":" + String(SIP_PASSWORD));
  String ha2 = getMD5("REGISTER:" + server_uri);
  String response = "";

  if (qop.length() > 0) {
    response = getMD5(ha1 + ":" + nonce + ":" + nc + ":" + cnonce + ":" + qop + ":" + ha2);
  } else {
    response = getMD5(ha1 + ":" + nonce + ":" + ha2);
  }

  String reg = "REGISTER " + server_uri + " SIP/2.0\r\n";
  reg += "Via: SIP/2.0/UDP " + local_ip + ":" + String(SIP_PORT) + ";rport;branch=z9hG4bK-reg-" + String(reg_cseq) + "\r\n";
  reg += "From: <sip:" + String(SIP_USER) + "@" + String(SIP_SERVER_IP) + ">;tag=" + reg_from_tag + "\r\n";
  reg += "To: <sip:" + String(SIP_USER) + "@" + String(SIP_SERVER_IP) + ">\r\n";
  reg += "Call-ID: " + reg_call_id + "\r\n";
  reg += "CSeq: " + String(reg_cseq) + " REGISTER\r\n";
  reg += "Contact: <sip:" + String(SIP_USER) + "@" + local_ip + ":" + String(SIP_PORT) + ">\r\n";
  
  reg += "Authorization: Digest username=\"" + String(SIP_USER) + "\", realm=\"" + realm + "\", nonce=\"" + nonce + "\", uri=\"" + server_uri + "\", response=\"" + response + "\"";
  if (opaque.length() > 0) {
    reg += ", opaque=\"" + opaque + "\"";
  }
  if (qop.length() > 0) {
    reg += ", qop=" + qop + ", nc=" + nc + ", cnonce=\"" + cnonce + "\"";
  }
  reg += ", algorithm=MD5\r\n";
  
  reg += "Max-Forwards: 70\r\n";
  reg += "Expires: 120\r\n";
  reg += "User-Agent: ESP32-ETH01-EVO\r\n";
  reg += "Content-Length: 0\r\n\r\n";

  udpSip.beginPacket(SIP_SERVER_IP, SIP_SERVER_PORT);
  udpSip.print(reg);
  udpSip.endPacket();

  Serial.println("[SIP] >>> Sent Authenticated REGISTER request.");
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] DM9051 Ethernet Started");
      ETH.setHostname("eth01-evo-voip");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Link UP");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println(" Mbps");
      
      ethernet_connected = true;
      udpSip.begin(SIP_PORT);
      udpRtpRx.begin(RTP_PORT_LOCAL);
      Serial.println("[SIP] Listening on port 5060...");

      if (ENABLE_SIP_REGISTER) {
        sendSipRegisterInitial();
      } else {
        Serial.println("[SIP] Running in Direct IP / Peer Intercom Mode.");
      }
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Link DOWN");
      ethernet_connected = false;
      in_call = false;
      sip_registered = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Ethernet Stopped");
      ethernet_connected = false;
      in_call = false;
      sip_registered = false;
      break;
    default:
      break;
  }
}

void handleSipMessages() {
  int packetSize = udpSip.parsePacket();
  if (packetSize) {
    char packetBuffer[1024];
    int len = udpSip.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
    }
    String msg = String(packetBuffer);
    IPAddress sender_ip = udpSip.remoteIP();
    
    // 1. Handle SIP Digest Authentication Challenge from PBX (401 / 407)
    if (msg.startsWith("SIP/2.0 401") || msg.startsWith("SIP/2.0 407")) {
      Serial.println("\n[SIP] <<< Received 401/407 Authentication Challenge");
      String auth_header = extractString(msg, "WWW-Authenticate: Digest ", "\r\n");
      if (auth_header == "") {
        auth_header = extractString(msg, "Proxy-Authenticate: Digest ", "\r\n");
      }
      
      String realm  = extractParam(auth_header, "realm=");
      String nonce  = extractParam(auth_header, "nonce=");
      String opaque = extractParam(auth_header, "opaque=");
      String qop    = extractParam(auth_header, "qop=");
      
      if (realm.length() > 0 && nonce.length() > 0) {
        sendSipRegisterAuth(realm, nonce, opaque, qop);
      }
    }
    // 2. Handle 200 OK Response for Registration
    else if (msg.startsWith("SIP/2.0 200 OK") && msg.indexOf("CSeq: ") != -1 && msg.indexOf("REGISTER") != -1) {
      sip_registered = true;
      last_reg_time = millis();
      Serial.printf("\n[SIP] *** SUCCESS: Registered with PBX Server (%s) as Extension %s ***\n\n", SIP_SERVER_IP, SIP_USER);
    }
    // 3. Handle Incoming Call (INVITE) - Ringing & Auto-Answer Mode
    else if (msg.startsWith("INVITE")) {
      remote_ip = sender_ip;
      
      String from_hdr = extractString(msg, "From: ", "\r\n");
      String caller_id = extractString(from_hdr, "<sip:", ">");
      if (caller_id == "") caller_id = from_hdr;

      Serial.println("\n=======================================================");
      Serial.printf("[SIP] 🔔 INCOMING CALL / RINGING from: %s\n", caller_id.c_str());
      Serial.println("[SIP] 📞 AUTO-ANSWER MODE ACTIVATED!");
      Serial.println("=======================================================");
      
      call_id = extractString(msg, "Call-ID: ", "\r\n");
      if(call_id == "") call_id = extractString(msg, "i: ", "\r\n");
      
      String via = extractString(msg, "Via: ", "\r\n");
      String from = extractString(msg, "From: ", "\r\n");
      String to = extractString(msg, "To: ", "\r\n");
      String cseq = extractString(msg, "CSeq: ", "\r\n");
      String local_ip = ETH.localIP().toString();

      // Step 1: Send SIP 180 Ringing
      String ringing_resp = "SIP/2.0 180 Ringing\r\n";
      ringing_resp += "Via: " + via + "\r\n";
      ringing_resp += "From: " + from + "\r\n";
      ringing_resp += "To: " + to + ";tag=esp32voip\r\n";
      ringing_resp += "Call-ID: " + call_id + "\r\n";
      ringing_resp += "CSeq: " + cseq + "\r\n";
      ringing_resp += "User-Agent: ESP32-ETH01-EVO\r\n";
      ringing_resp += "Content-Length: 0\r\n\r\n";

      udpSip.beginPacket(remote_ip, udpSip.remotePort());
      udpSip.print(ringing_resp);
      udpSip.endPacket();

      Serial.println("[SIP] >>> Sent 180 Ringing to Caller.");
      delay(150);

      // Step 2: Send SIP 200 OK (Auto-Answer + SDP)
      String m_audio_line = extractString(msg, "m=audio ", " ");
      if (m_audio_line.length() > 0) {
        remote_rtp_port = m_audio_line.toInt();
      }
      
      Serial.printf("[SIP] Remote RTP Destination: %s:%d\n", remote_ip.toString().c_str(), remote_rtp_port);

      String sdp = "v=0\r\n";
      sdp += "o=esp32 12345 12345 IN IP4 " + local_ip + "\r\n";
      sdp += "s=Talk\r\n";
      sdp += "c=IN IP4 " + local_ip + "\r\n";
      sdp += "t=0 0\r\n";
      sdp += "m=audio " + String(RTP_PORT_LOCAL) + " RTP/AVP 0\r\n"; // Payload Type 0 = PCMU (u-law)
      sdp += "a=rtpmap:0 PCMU/8000\r\n";
      sdp += "a=sendrecv\r\n";

      String response = "SIP/2.0 200 OK\r\n";
      response += "Via: " + via + "\r\n";
      response += "From: " + from + "\r\n";
      response += "To: " + to + ";tag=esp32voip\r\n";
      response += "Call-ID: " + call_id + "\r\n";
      response += "CSeq: " + cseq + "\r\n";
      response += "Contact: <sip:" + String(SIP_USER) + "@" + local_ip + ":" + String(SIP_PORT) + ">\r\n";
      response += "Content-Type: application/sdp\r\n";
      response += "Content-Length: " + String(sdp.length()) + "\r\n\r\n";
      response += sdp;

      udpSip.beginPacket(remote_ip, udpSip.remotePort());
      udpSip.print(response);
      udpSip.endPacket();
      
      Serial.println("[SIP] >>> Sent 200 OK - Call Auto-Answered! Audio streaming active.");
      in_call = true;
    } 
    // 4. Handle End Call (BYE)
    else if (msg.startsWith("BYE")) {
      Serial.println("\n=======================================================");
      Serial.println("[SIP] 📴 CALL ENDED (BYE Received)");
      Serial.println("=======================================================");
      
      String via = extractString(msg, "Via: ", "\r\n");
      String from = extractString(msg, "From: ", "\r\n");
      String to = extractString(msg, "To: ", "\r\n");
      String cseq = extractString(msg, "CSeq: ", "\r\n");
      
      String response = "SIP/2.0 200 OK\r\n";
      response += "Via: " + via + "\r\n";
      response += "From: " + from + "\r\n";
      response += "To: " + to + "\r\n";
      response += "Call-ID: " + call_id + "\r\n";
      response += "CSeq: " + cseq + "\r\n";
      response += "Content-Length: 0\r\n\r\n";

      udpSip.beginPacket(sender_ip, udpSip.remotePort());
      udpSip.print(response);
      udpSip.endPacket();
      
      in_call = false;
    }
  }
}

// RTP Header variables
uint16_t rtp_seq = 0;
uint32_t rtp_timestamp = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==============================================");
  Serial.println(" ETH01 EVO (ESP32-C3) VoIP Intercom Starting");
  Serial.println(" Audio Engine: Low-Latency Full-Duplex");
  Serial.println("==============================================");

  setup_i2s();
  Serial.println("[I2S] Audio Interface Initialized.");

  WiFi.onEvent(WiFiEvent);
  Serial.println("[ETH] Initializing DM9051 Ethernet...");
  if (!ETH.begin()) {
    Serial.println("[ETH] ERROR: Ethernet Initialization failed!");
  }
}

void loop() {
  if (ethernet_connected) {
    handleSipMessages();

    // Re-register every 60 seconds if SIP Registration is enabled
    if (ENABLE_SIP_REGISTER && millis() - last_reg_time > 60000) {
      last_reg_time = millis();
      sendSipRegisterInitial();
    }
    
    if (in_call) {
      // 1. SPEAKER RX: Empty ALL pending incoming RTP packets to prevent jitter buffer stutter
      int rtpSize;
      while ((rtpSize = udpRtpRx.parsePacket()) > 12) {
        uint8_t rtp_buffer[500];
        int read_bytes = udpRtpRx.read(rtp_buffer, sizeof(rtp_buffer));
        if (read_bytes <= 12) continue;
        
        int payload_len = read_bytes - 12;
        int32_t pcm_out_32[payload_len * 2]; // 32-bit Stereo L+R
        
        for (int i = 0; i < payload_len; i++) {
          int16_t pcm16 = ulaw_decode_table[rtp_buffer[12 + i]];
          
          // Apply speaker gain & anti-clipping limiter
          int32_t pcm_scaled = (int32_t)(pcm16 * SPEAKER_GAIN);
          if (pcm_scaled > 32767) pcm_scaled = 32767;
          if (pcm_scaled < -32768) pcm_scaled = -32768;
          
          int32_t val32 = ((int32_t)pcm_scaled) << 16;
          pcm_out_32[i * 2]     = val32; // Left channel
          pcm_out_32[i * 2 + 1] = val32; // Right channel
        }
        
        size_t bytes_written = 0;
        i2s_write(I2S_PORT_NUM, pcm_out_32, payload_len * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);
      }

      // 2. MIC TX: Read audio from INMP441, encode to G.711 PCMU, Send RTP
      size_t bytes_read = 0;
      int32_t mic_samples[320]; // 160 Left + 160 Right (Stereo 32-bit slots)
      
      // Read audio with short timeout (10ms) to maintain smooth speaker processing
      i2s_read(I2S_PORT_NUM, &mic_samples, sizeof(mic_samples), &bytes_read, pdMS_TO_TICKS(10));
      
      if (bytes_read == sizeof(mic_samples)) {
        int num_stereo_samples = bytes_read / sizeof(int32_t); // 320
        int num_mono_samples = num_stereo_samples / 2; // 160
        uint8_t rtp_packet[12 + 160]; // 12 byte RTP header + payload
        
        // RTP Header
        rtp_packet[0] = 0x80; // Version 2
        rtp_packet[1] = 0x00; // Payload type 0 (PCMU)
        rtp_packet[2] = (rtp_seq >> 8) & 0xFF;
        rtp_packet[3] = rtp_seq & 0xFF;
        rtp_packet[4] = (rtp_timestamp >> 24) & 0xFF;
        rtp_packet[5] = (rtp_timestamp >> 16) & 0xFF;
        rtp_packet[6] = (rtp_timestamp >> 8) & 0xFF;
        rtp_packet[7] = rtp_timestamp & 0xFF;
        // SSRC (static ID)
        rtp_packet[8] = 0x12; rtp_packet[9] = 0x34; rtp_packet[10] = 0x56; rtp_packet[11] = 0x78;
        
        // Encode payload
        int16_t pcm_max = 0;
        for (int i = 0; i < num_mono_samples; i++) {
          int32_t left_pcm  = mic_samples[i * 2] >> 16;
          int32_t right_pcm = mic_samples[i * 2 + 1] >> 16;
          
          int32_t pcm32 = (abs(left_pcm) > abs(right_pcm)) ? left_pcm : right_pcm;
          
          pcm32 = pcm32 * MIC_GAIN; // Digital Mic Gain
          if (pcm32 > 32767) pcm32 = 32767;
          if (pcm32 < -32768) pcm32 = -32768;
          
          int16_t pcm = (int16_t)pcm32;
          if (abs(pcm) > pcm_max) {
            pcm_max = abs(pcm);
          }
          
          rtp_packet[12 + i] = pcm_to_ulaw(pcm);
        }
        
        // Print ASCII volume bar graph every 200ms (10 packets)
        if (rtp_seq % 10 == 0) {
          int num_bars = (pcm_max * 30) / 32768;
          String meter = "[";
          for (int b = 0; b < 30; b++) {
            meter += (b < num_bars) ? "#" : " ";
          }
          meter += "]";
          Serial.printf("[MIC] %s Vol: %d\n", meter.c_str(), pcm_max);
        }
        
        // Send RTP packet to remote party
        udpRtpRx.beginPacket(remote_ip, remote_rtp_port);
        udpRtpRx.write(rtp_packet, 12 + num_mono_samples);
        udpRtpRx.endPacket();
        
        rtp_seq++;
        rtp_timestamp += num_mono_samples;
      }
    }
  }
}
