# WT32-ETH01 & ETH01-EVO (ESP32 / ESP32-C3) VoIP SIP Intercom System

A high-performance, low-latency, full-duplex **VoIP / SIP Intercom system** built for Espressif microcontrollers including the **WT32-ETH01** (ESP32 Classic with onboard LAN8720 RMII Ethernet) and the **ETH01-EVO** (ESP32-C3 with DM9051 SPI Ethernet).

This project turns hardware boards into standalone SIP endpoints capable of auto-answering incoming calls, registering with Asterisk PBX servers via MD5 Digest Authentication, and establishing two-way real-time audio communication using G.711 (PCMU u-law) RTP audio stream.

---

## 🌟 Key Features

* **Full-Duplex Audio Engine:** Real-time bi-directional audio streaming at 8kHz sample rate using standard G.711 PCMU (u-law) codec.
* **Dual Board & Hardware Support:**
  * **WT32-ETH01 (ESP32):** Onboard LAN8720 RMII Ethernet PHY + dual hardware I2S controllers.
  * **ETH01-EVO (ESP32-C3):** Onboard DM9051 SPI Ethernet controller + custom single-I2S full-duplex clock matrix routing.
* **Dual Operation Modes:**
  * **Registered SIP Mode:** Full SIP Registration flow with 401/407 Digest MD5 Authentication.
  * **Direct IP / Peer Mode:** Direct peer-to-peer SIP calls without requiring a PBX server.
* **Auto-Answer & Ringing Flow:** Explicit `180 Ringing` notification followed by `200 OK` + SDP negotiation upon receiving an `INVITE`.
* **Audio Hardware Integration:** Works with INMP441 I2S Digital Microphone and MAX98357A I2S Class-D Audio Amplifier.
* **Diagnostic Tools:** Integrated live ASCII volume meter on the Serial console (115200 baud) and standalone I2S loopback/test utility.

---

## 📐 Hardware Schematic & Pinout

The repository includes the official hardware schematic:  
📄 [`WT32-ETH01_V1.4.schematic.pdf`](./WT32-ETH01_V1.4.schematic.pdf)

### 1. WT32-ETH01 (ESP32 Classic) Pinout

On standard ESP32 boards, dual hardware I2S ports (`I2S_NUM_0` for Mic, `I2S_NUM_1` for Speaker) are used to isolate clock lines and avoid conflict with LAN8720 Ethernet PHY pins.

| Peripheral | Signal | WT32-ETH01 Pin | Description |
| :--- | :--- | :--- | :--- |
| **INMP441 Mic** | SCK / BCLK | `GPIO 32` | Bit Clock |
| | WS / LRC | `GPIO 33` | Word Select (Left/Right Clock) |
| | SD / DOUT | `GPIO 36` | Data Out (*Input-only pin*) |
| | L/R | `GND` | Left Channel |
| **MAX98357A Amp** | BCLK | `GPIO 14` | Bit Clock |
| | LRC / WS | `GPIO 15` | Word Select |
| | DIN | `GPIO 4` | Data In |
| **LAN8720 PHY** | MDC | `GPIO 23` | Management Data Clock |
| | MDIO | `GPIO 18` | Management Data I/O |
| | CLK_IN | `GPIO 0` | 50MHz Clock Input |
| | Power | `GPIO 16` | PHY Power Enable |

---

### 2. ETH01-EVO (ESP32-C3) Pinout

The ESP32-C3 features a single hardware I2S peripheral (`I2S_NUM_0`). The code detaches USB-JTAG from `GPIO 18`/`GPIO 19` and uses the ESP32-C3 ROM GPIO Matrix to route master clocks to the microphone while keeping independent data lines.

| Peripheral | Signal | ETH01-EVO Pin | Description |
| :--- | :--- | :--- | :--- |
| **INMP441 Mic** | BCLK / SCK | `GPIO 18` | Microphone Bit Clock (routed via GPIO Matrix) |
| | WS / LRC | `GPIO 19` | Microphone Word Select (routed via GPIO Matrix) |
| | SD / DOUT | `GPIO 1` | Microphone Data Input |
| **MAX98357A Amp** | BCLK | `GPIO 5` | Speaker Bit Clock |
| | LRC / WS | `GPIO 2` | Speaker Word Select |
| | DIN | `GPIO 4` | Speaker Data Output |
| **DM9051 Ethernet**| CS | `GPIO 9` | SPI Chip Select |
| | IRQ | `GPIO 8` | SPI Interrupt |
| | RST | `GPIO 6` | SPI Reset |
| | SCK | `GPIO 7` | SPI Clock |
| | MISO | `GPIO 3` | SPI Master In Slave Out |
| | MOSI | `GPIO 10` | SPI Master Out Slave In |

---

## 📦 Required Libraries & Software Setup

All required networking, socket, and peripheral drivers are **natively included in the Espressif ESP32 Core for Arduino**. No third-party library manager downloads are required.

### Arduino Board Manager Dependencies:
* **Board Core:** `esp32` by **Espressif Systems** (Version `2.0.11` or `3.x` recommended).
* **Board Selection:**
  * For WT32-ETH01: Select `ESP32 Wrover Module` (or `WT32-ETH01`).
  * For ETH01-EVO (ESP32-C3): Select `ESP32C3 Dev Module` (or `ETH01-EVO`).

### Included Native Core Libraries & Headers:
* `<ETH.h>` – Hardware Ethernet driver (LAN8720 RMII & DM9051 SPI).
* `<WiFi.h>` & `<WiFiUdp.h>` – Network sockets and UDP packet handling for SIP and RTP.
* `"driver/i2s.h"` – ESP-IDF I2S digital audio controller driver.
* `<MD5Builder.h>` – Digest authentication hash generation for SIP registration.
* `<esp_rom_gpio.h>` & `<soc/gpio_sig_map.h>` – Low-level GPIO matrix signal routing for ESP32-C3.

---

## 📂 Project Repository Structure

```
.
├── README.md                      # Comprehensive project documentation
├── WT32-ETH01_V1.4.schematic.pdf   # Hardware schematic diagram
├── VOIP_C3_EVO.ino                # Main unified VoIP sketch for ESP32 & ESP32-C3
├── VOIP_ETH01_EVO/
│   └── VOIP_ETH01_EVO.ino         # Dedicated ETH01-EVO (ESP32-C3 + DM9051) sketch
├── I2S_Test/
│   └── I2S_Test.ino               # Hardware diagnostic loopback test for Mic & Speaker
└── asterisk_configs/
    ├── pjsip.conf                 # Modern Asterisk PJSIP configuration
    ├── sip.conf                   # Legacy Asterisk chan_sip configuration
    └── extensions.conf            # Asterisk Dialplan routing logic
```

---

## 📞 Asterisk PBX Setup Guide

To connect the ESP32 Intercom to an Asterisk PBX Server, use the provided configuration files located in the `asterisk_configs/` directory.

### 1. Modern PJSIP Configuration (`asterisk_configs/pjsip.conf`)

```ini
[global]
type=global
user_agent=Asterisk PBX

[transport-udp]
type=transport
protocol=udp
bind=0.0.0.0:5060

; --- Softphone Extension 100 ---
[100]
type=auth
auth_type=userpass
username=100
password=1234

[100]
type=aor
max_contacts=1

[100]
type=endpoint
context=intercom
disallow=all
allow=ulaw
auth=100
aors=100

; --- ESP32 Intercom Extension 200 ---
[200]
type=auth
auth_type=userpass
username=200
password=1234

[200]
type=aor
max_contacts=1

[200]
type=endpoint
context=intercom
disallow=all
allow=ulaw
auth=200
aors=200
```

### 2. Asterisk Dialplan (`asterisk_configs/extensions.conf`)

```ini
[intercom]
exten => 100,1,NoOp(Calling Softphone Extension 100)
 exten => 100,n,Dial(PJSIP/100,30)
 exten => 100,n,Hangup()

exten => 200,1,NoOp(Calling ESP32 Intercom Extension 200)
 exten => 200,n,Dial(PJSIP/200,30)
 exten => 200,n,Hangup()
```

---

## ⚡ Flashing & Upload Instructions

Because the WT32-ETH01 and ETH01-EVO boards do not have an onboard USB-to-Serial converter, code must be uploaded using an external **USB-to-TTL Serial Adapter** (FT232RL, CP2102, or CH340).

### 1. Serial Programmer Wiring

| FTDI / USB-TTL Adapter | Board Pin | Notes |
| :--- | :--- | :--- |
| **TX** | **RX0** | Serial Data Receiver |
| **RX** | **TX0** | Serial Data Transmitter |
| **5V / 3.3V** | **5V / 3V3** | Power supply |
| **GND** | **GND** | Common Ground |

### 2. How to Enter Flashing Mode (Bootloader)

1. Connect **IO0** pin to **GND**.
2. Press the Reset button or power-cycle the board.
3. Open Arduino IDE and click **Upload**.
4. Once uploading completes ("Done uploading"), **disconnect IO0 from GND**.
5. Power-cycle the board. Open Serial Monitor at **115200 baud** to view system initialization logs and the ASCII audio volume graph.

---

## 🔧 Audio Diagnostics & Troubleshooting

* **Testing Audio Hardware First:** Before compiling the SIP stack, flash `I2S_Test/I2S_Test.ino` to verify microphone input levels and speaker output.
* **Volume & Mic Boost Adjustment:** In `VOIP_ETH01_EVO.ino`, adjust gain values as needed:
  ```cpp
  float SPEAKER_GAIN = 1.0f; // Multiplier for speaker volume
  int   MIC_GAIN     = 8;    // Digital boost for INMP441 mic (4, 8, 16)
  ```
* **Audio Noise / Buzzing:** Ensure the MAX98357A amplifier and INMP441 microphone share a short, common ground connection with the ESP32 board.
