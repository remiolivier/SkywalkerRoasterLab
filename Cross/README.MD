Cross_Skywalker v1.1 Firmware (WROOM Edition)
Features:

Dual temperature input support

Compatible with Artisan via USB, Bluetooth, and WiFi

Compatible with HiBean via BLE (Bluetooth Low Energy)

Sensor Pin Configuration:

MAX6675 Thermocouple Module

SCK → GPIO 18

CS → GPIO 5

SO → GPIO 19

Serial Communication:

TX → GPIO 16

RX → GPIO 17

WiFi Setup Instructions
Power on the ESP32 development board.

On your mobile phone or computer (ensure mobile data is disabled), search for and connect to the WiFi network named:
ESP32_Roaster

Open a browser and go to:
http://192.168.4.1

Enter your WiFi SSID (note: 5GHz networks are not supported) and password.

The board will save the credentials, reboot, and automatically connect to your specified network (one-time setup only).

Using with Artisan
For Bluetooth or USB connections:
Simply select the appropriate COM port or Bluetooth device in Artisan.

For WiFi connections:

Use the included Artisan(wifi).aset configuration file to connect.

Alternatively, refer to the provided WebSocket settings and apply them to your own Artisan configuration.

Notes
Both Artisan and HiBean can be run independently or simultaneously.

WiFi connectivity is only available in firmware builds that support it (e.g., Cross_Skywalker_v1.1.bin).


