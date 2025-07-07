# Cross_Skywalker v1.6.2 Firmware (ESP32 WROOM Edition)

Firmware: `Cross_Skywalker_v1.6.2.bin`  
👨‍💻 **Created by: Bear**

---

## ✅ Features

- Dual temperature input support
- Supports **Artisan** via:
  - USB
  - Bluetooth
  - WiFi
- Supports **HiBean** via:
  - BLE (Bluetooth Low Energy)
- Artisan & HiBean can run **independently or simultaneously**

---

## 📌 Pin Configuration

### Thermocouple (MAX6675)

| Signal | GPIO |
|--------|------|
| SCK    | 18   |
| CS     | 5    |
| SO     | 19   |

### Serial Communication

| Label | GPIO |
|-------|------|
| TX    | 16   |
| RX    | 17   |

---

## 📶 WiFi Setup Instructions

1. Power on your ESP32 development board.
2. On your mobile device or computer:
   - Turn **off** mobile data (important).
   - Connect to the WiFi network:  
     **`ESP32_Roaster`**
3. Open your browser and go to:  
   **`http://192.168.4.1`**
4. Enter your home WiFi:
   - **SSID** (⚠️ 5GHz not supported)
   - **Password**
5. The device will save the credentials, reboot, and auto-connect to your network.

> 🔁 This setup is only required once.

---

## 🔧 Using with Artisan

### USB or Bluetooth (For `Cross_Skywalker_v1.1.bin` Only)

- Select the corresponding COM port or Bluetooth device in Artisan.

### WiFi (WebSocket)

- Use the included config file:  
  **`Artisan(wifi).aset`**
- Or manually configure WebSocket settings based on the included documentation.

---

## 📁 Included Files

- `Cross_Skywalker_v1.1.bin`
- `Artisan(wifi).aset`
- WebSocket connection guide (optional)

---

## 🧠 Notes

- Only one-time WiFi setup is needed unless credentials change.
- Artisan and HiBean can be used at the same time without conflict.
- Make sure to use 2.4GHz WiFi only.

---

© 2025 by **Bear**
