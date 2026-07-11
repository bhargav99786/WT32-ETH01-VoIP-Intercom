# WT32-ETH01 Modbus TCP Server (8-Channel Switch Monitor)

This project turns a **WT32-ETH01 V1.4** (ESP32 with onboard LAN8720 Ethernet PHY) into a robust Modbus TCP Server. It reads the state of 8 physical switches and exposes them as Holding Registers over Ethernet, which can be monitored via any Modbus TCP client (like QModMaster or QModBus).

## 🛠️ Hardware Requirements
* **Board:** WT32-ETH01 V1.4
* **Network:** Standard RJ45 Ethernet connection
* **Switches:** 8x Switches or digital sensors
* **Programmer:** USB-to-TTL Serial Adapter (FTDI) for uploading code

## ⚙️ Software & Library Versions
* **Arduino IDE:** 2.x (Recommended)
* **ESP32 Core Version:** 2.0.11 or later (Board Manager: `esp32 by Espressif Systems`)
* **Libraries:**
  * `eModbus` by eModbus Team (Used for Modbus TCP handling)
  * Built-in `ETH.h` (For LAN8720 PHY)

## 🔌 Connection Diagram (Switch Inputs)

The 8 switches are connected to the following GPIO pins on the WT32-ETH01:

| Modbus Register | Pin Number | Note / Requirement |
| :--- | :--- | :--- |
| **Register 100** | GPIO 2 | Internal pull-up/down available |
| **Register 101** | GPIO 4 | Internal pull-up/down available |
| **Register 102** | GPIO 12 | Internal pull-up/down available |
| **Register 103** | GPIO 14 | Internal pull-up/down available |
| **Register 104** | GPIO 15 | Internal pull-up/down available |
| **Register 105** | GPIO 36 | **Input Only** - *Must use external pull-up/down resistor* |
| **Register 106** | GPIO 39 | **Input Only** - *Must use external pull-up/down resistor* |
| **Register 107** | GPIO 35 | **Input Only** - *Must use external pull-up/down resistor* |

> ⚠️ **IMPORTANT:** Pins 34, 35, 36, and 39 on the ESP32 do NOT have internal pull resistors. You MUST add physical resistors (e.g., 10kΩ) to GND or 3.3V on these pins so they don't float and give false readings.

## 🚀 How to Upload Code to WT32-ETH01
Because the WT32-ETH01 does **not** have an onboard USB port, you must upload the code using a USB-to-TTL converter (like an FT232RL or CP2102).

### 1. Wiring the Programmer
| FTDI Programmer | WT32-ETH01 |
| :--- | :--- |
| 5V (or 3.3V) | 5V (or 3V3) |
| GND | GND |
| TX | RX0 |
| RX | TX0 |

### 2. Enter Bootloader Mode (Flash Mode)
To put the board into programming mode:
1. Connect the **IO0** pin to **GND** on the WT32-ETH01.
2. Power cycle the board (or press the EN/Reset button if you have one wired).
3. The board is now ready to receive code.

### 3. Upload via Arduino IDE
1. Open Arduino IDE.
2. Select Board: **ESP32 Wrover Module** (or `WT32-ETH01` if available in your core version).
3. Click **Upload**.
4. Once it says "Done uploading", **disconnect IO0 from GND**.
5. Power cycle the board. It will now run your code!

## 📡 Modbus Client Configuration (QModMaster / QModBus)
To view the data on your computer:
1. Ensure your PC and the WT32-ETH01 are on the same network.
2. Open your Modbus Client.
3. **Modbus Mode:** `TCP`
4. **IP Address:** Check the Arduino Serial Monitor for the IP assigned via DHCP.
5. **Port:** `502`
6. **Slave ID:** `1`
7. **Function Code:** `Read Holding Registers (0x03)`
8. **Start Address:** `100` (Make sure format is Decimal / `Dec`)
9. **Number of Registers:** `8`
