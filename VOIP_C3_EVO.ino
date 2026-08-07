/*
 * WT32-ETH01 VoIP / SIP Intercom Project (Direct IP)
 * 
 * Hardware:
 * - WT32-ETH01 (ESP32 + LAN8720)
 * - INMP441 I2S Microphone (Input)
 * - MAX98357A I2S Amplifier (Output)
 * - USB-to-TTL (TX to RX0, RX to TX0) for Serial Monitor
 * 
 * I2S Pin Mapping (Split Clocks):
 *   Mic SCK   -> GPIO 32
 *   Mic WS    -> GPIO 33
 *   Mic SD    -> GPIO 36 (Input Only)
 *   Amp BCLK  -> GPIO 14
 *   Amp LRC   -> GPIO 15
 *   Amp DIN   -> GPIO 4
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
#include <WiFiUdp.h>
#include "driver/i2s.h"
#ifndef I2S_NUM_1
  #include <esp_rom_gpio.h>
  #include <soc/gpio_sig_map.h>
  #include <soc/usb_serial_jtag_reg.h>
  #include <soc/gpio_periph.h>
  #include <driver/gpio.h>
#endif

// --- I2S Configuration ---
#ifdef I2S_NUM_1
  // Dual I2S Ports available (Standard ESP32, e.g. WT32-ETH01)
  #define I2S_PORT_RX I2S_NUM_0 // Microphone
  #define I2S_PORT_TX I2S_NUM_1 // Speaker

  // --- Microphone (RX) Pins ---
  #define I2S_RX_BCLK_PIN  32
  #define I2S_RX_LRC_PIN   33
  #define I2S_RX_DOUT_PIN  36 // Microphone Data In

  // --- Speaker (TX) Pins ---
  #define I2S_TX_BCLK_PIN  14
  #define I2S_TX_LRC_PIN   15
  #define I2S_TX_DIN_PIN   4  // Speaker Data Out
#else
  // Single I2S Port (ESP32-C3 / ETH01 EVO C3 based board)
  #define I2S_PORT_RX I2S_NUM_0
  #define I2S_PORT_TX I2S_NUM_0

  // Separate Pins for Microphone (RX)
  #define I2S_RX_BCLK_PIN  18 // Mic SCK / BCLK
  #define I2S_RX_LRC_PIN   19 // Mic WS / LRC
  #define I2S_RX_DOUT_PIN  1  // Mic SD / Data In

  // Separate Pins for Speaker (TX)
  #define I2S_TX_BCLK_PIN  5 // Speaker BCLK
  #define I2S_TX_LRC_PIN   2  // Speaker LRC
  #define I2S_TX_DIN_PIN   4  // Speaker DIN / Data Out
#endif

#define SAMPLE_RATE   8000 // Standard for VoIP (G.711)

WiFiUDP udpRtpTx;
WiFiUDP udpRtpRx;
WiFiUDP udpSip;

const int SIP_PORT = 5060;
const int RTP_PORT_LOCAL = 7078;
int remote_rtp_port = 8000; // Default, usually parsed from SDP
IPAddress remote_ip;

bool eth_connected = false;
bool in_call = false;
String call_id = "";
String from_tag = "";

// --- G.711 u-law compression tables ---
// Very basic u-law encoding/decoding lookup for 8kHz 16-bit audio
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

void setup_i2s() {
#ifdef I2S_NUM_1
  // --- Standard Dual-I2S Setup (ESP32) ---
  i2s_config_t i2s_config_rx = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Read stereo to avoid ONLY_LEFT interleaving bugs
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 320, // 160 left + 160 right
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_rx = {
      .bck_io_num = I2S_RX_BCLK_PIN,
      .ws_io_num = I2S_RX_LRC_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_RX_DOUT_PIN
  };

  i2s_driver_install(I2S_PORT_RX, &i2s_config_rx, 0, NULL);
  i2s_set_pin(I2S_PORT_RX, &pin_config_rx);
  i2s_start(I2S_PORT_RX);

  i2s_config_t i2s_config_tx = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 160,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_tx = {
      .bck_io_num = I2S_TX_BCLK_PIN,
      .ws_io_num = I2S_TX_LRC_PIN,
      .data_out_num = I2S_TX_DIN_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT_TX, &i2s_config_tx, 0, NULL);
  i2s_set_pin(I2S_PORT_TX, &pin_config_tx);
  i2s_start(I2S_PORT_TX);
#else
  // --- Single-I2S Full Duplex Setup with Independent Pins (ESP32-C3 / ETH01 EVO C3) ---
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
      .dma_buf_len = 320,
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

  i2s_driver_install(I2S_PORT_RX, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT_RX, &pin_config);

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

  i2s_start(I2S_PORT_RX);
#endif
}

void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Ethernet Started");
      ETH.setHostname("wt32-voip");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Link UP");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] IPv4: ");
      Serial.println(ETH.localIP());
      udpSip.begin(SIP_PORT);
      udpRtpRx.begin(RTP_PORT_LOCAL);
      Serial.println("[SIP] Listening for Direct IP Calls on port 5060...");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Link DOWN");
      eth_connected = false;
      in_call = false;
      break;
    default:
      break;
  }
}

void setup() {
  // Use standard Serial baud for the USB-to-TTL connection
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=========================================");
  Serial.println(" WT32-ETH01 VoIP Intercom Initialization");
  Serial.println(" (USB-to-TTL Serial Monitor connected)");
  Serial.println("=========================================");

  setup_i2s();
  Serial.println("[I2S] Microphone & Speaker Initialized.");

  WiFi.onEvent(onEthEvent);
  ETH.begin(ETH_PHY_ADDR, ETH_POWER_PIN, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
}

// Function to extract text between two strings
String extractString(String source, String start, String end) {
  int startIndex = source.indexOf(start);
  if (startIndex == -1) return "";
  startIndex += start.length();
  int endIndex = source.indexOf(end, startIndex);
  if (endIndex == -1) return source.substring(startIndex);
  return source.substring(startIndex, endIndex);
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
    remote_ip = udpSip.remoteIP();
    
    if (msg.startsWith("INVITE")) {
      Serial.println("\n[SIP] <<< INVITE received!");
      
      // Extract Call-ID
      call_id = extractString(msg, "Call-ID: ", "\r\n");
      if(call_id == "") call_id = extractString(msg, "i: ", "\r\n"); // short form
      
      // Extract remote RTP port from SDP
      String m_audio_line = extractString(msg, "m=audio ", " ");
      if (m_audio_line.length() > 0) {
        remote_rtp_port = m_audio_line.toInt();
      }
      
      Serial.printf("[SIP] Remote RTP Port: %d\n", remote_rtp_port);

      // We auto-answer the call with a 200 OK + SDP
      String local_ip = ETH.localIP().toString();
      
      // Build SDP Body
      String sdp = "v=0\r\n";
      sdp += "o=esp32 12345 12345 IN IP4 " + local_ip + "\r\n";
      sdp += "s=Talk\r\n";
      sdp += "c=IN IP4 " + local_ip + "\r\n";
      sdp += "t=0 0\r\n";
      sdp += "m=audio " + String(RTP_PORT_LOCAL) + " RTP/AVP 0\r\n"; // 0 = PCMU (u-law)
      sdp += "a=rtpmap:0 PCMU/8000\r\n";
      sdp += "a=sendrecv\r\n";

      // Extract Via, From, To, CSeq for the response
      String via = extractString(msg, "Via: ", "\r\n");
      String from = extractString(msg, "From: ", "\r\n");
      String to = extractString(msg, "To: ", "\r\n");
      String cseq = extractString(msg, "CSeq: ", "\r\n");
      
      String response = "SIP/2.0 200 OK\r\n";
      response += "Via: " + via + "\r\n";
      response += "From: " + from + "\r\n";
      response += "To: " + to + ";tag=esp32voip\r\n";
      response += "Call-ID: " + call_id + "\r\n";
      response += "CSeq: " + cseq + "\r\n";
      response += "Contact: <sip:esp32@" + local_ip + ":" + String(SIP_PORT) + ">\r\n";
      response += "Content-Type: application/sdp\r\n";
      response += "Content-Length: " + String(sdp.length()) + "\r\n\r\n";
      response += sdp;

      udpSip.beginPacket(remote_ip, udpSip.remotePort());
      udpSip.print(response);
      udpSip.endPacket();
      
      Serial.println("[SIP] >>> Sent 200 OK with SDP.");
      in_call = true;
    } 
    else if (msg.startsWith("BYE")) {
      Serial.println("\n[SIP] <<< BYE received. Ending call.");
      
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

      udpSip.beginPacket(remote_ip, udpSip.remotePort());
      udpSip.print(response);
      udpSip.endPacket();
      
      in_call = false;
    }
  }
}

// RTP Header variables
uint16_t rtp_seq = 0;
uint32_t rtp_timestamp = 0;

void loop() {
  if (eth_connected) {
    handleSipMessages();
    
    if (in_call) {
      // 1. MIC TX: Read audio, encode to G.711 PCMU, Send RTP
      size_t bytes_read = 0;
      int32_t mic_samples[320]; // 160 Left + 160 Right (Stereo)
      
      // Block until we have exactly 320 stereo samples
      i2s_read(I2S_PORT_RX, &mic_samples, sizeof(mic_samples), &bytes_read, portMAX_DELAY);
      
      if (bytes_read == sizeof(mic_samples)) {
        int num_stereo_samples = bytes_read / 4; // 320
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
        // SSRC (random static ID)
        rtp_packet[8] = 0x12; rtp_packet[9] = 0x34; rtp_packet[10] = 0x56; rtp_packet[11] = 0x78;
        
        // Encode payload
        int16_t pcm_max = 0;
        for (int i = 0; i < num_mono_samples; i++) {
          // Extract 16-bit PCM from both Left (even) and Right (odd) channel slots
          int32_t left_pcm  = mic_samples[i * 2] >> 16;
          int32_t right_pcm = mic_samples[i * 2 + 1] >> 16;
          
          // Select active channel sample (whether INMP441 L/R pin is wired to GND or VCC)
          int32_t pcm32 = (abs(left_pcm) > abs(right_pcm)) ? left_pcm : right_pcm;
          
          // Safely boost volume (8x gain) and clip to 16-bit limits
          pcm32 = pcm32 * 8; 
          if (pcm32 > 32767) pcm32 = 32767;
          if (pcm32 < -32768) pcm32 = -32768;
          
          int16_t pcm = (int16_t)pcm32;
          
          if (abs(pcm) > pcm_max) {
            pcm_max = abs(pcm);
          }
          
          rtp_packet[12 + i] = pcm_to_ulaw(pcm);
        }
        
        // Print an ASCII volume meter every 200ms (10 packets)
        if (rtp_seq % 10 == 0) {
          int num_bars = (pcm_max * 30) / 32768; // Scale to 30 characters max
          String meter = "[";
          for (int b = 0; b < 30; b++) {
            if (b < num_bars) meter += "#";
            else meter += " ";
          }
          meter += "]";
          Serial.printf("[MIC] %s Vol: %d\n", meter.c_str(), pcm_max);
        }
        
        // We MUST use the same UDP socket for RX and TX to ensure the source port matches our SDP (7078)
        // Otherwise, Asterisk's Strict RTP will drop the packets!
        udpRtpRx.beginPacket(remote_ip, remote_rtp_port);
        udpRtpRx.write(rtp_packet, 12 + num_mono_samples);
        udpRtpRx.endPacket();
        
        rtp_seq++;
        rtp_timestamp += num_mono_samples;
      }
      
      // 2. SPEAKER RX: Receive RTP, decode PCMU, write to Amp
      int rtpSize = udpRtpRx.parsePacket();
      if (rtpSize > 12) { // Must be larger than RTP header
        uint8_t rtp_buffer[500];
        udpRtpRx.read(rtp_buffer, sizeof(rtp_buffer));
        
        int payload_len = rtpSize - 12; // Skip 12 byte header
        int16_t pcm_out[payload_len];
        
        for (int i = 0; i < payload_len; i++) {
          pcm_out[i] = ulaw_decode_table[rtp_buffer[12 + i]];
        }
        
        size_t bytes_written = 0;
#ifdef I2S_NUM_1
        i2s_write(I2S_PORT_TX, pcm_out, payload_len * 2, &bytes_written, portMAX_DELAY);
#else
        // For single port 32-bit stereo format on ESP32-C3, format 32-bit samples for speaker
        int32_t pcm_out_32[payload_len * 2]; // Left + Right channels
        for (int i = 0; i < payload_len; i++) {
          int32_t val32 = ((int32_t)pcm_out[i]) << 16;
          pcm_out_32[i * 2]     = val32; // Left
          pcm_out_32[i * 2 + 1] = val32; // Right
        }
        i2s_write(I2S_PORT_TX, pcm_out_32, payload_len * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);
#endif
      }
    }
  }
}
