# Trident for Skywalker

Combines BLE, USB and WiFi access to the Skywalker roaster in one controller.

# Supported boards:

[ESP32-S3 devkitc 1 (dual usb-c)](https://a.aliexpress.com/_EH8OGvc)

# BLE setup
Use HiBean to connect to Trident. Search for "Skywalker-Trident" when adding the roaster in HiBean app.

# USB setup
Use the left usb-c port (when the board is facing you, usb ports facing down), to connect it to your computer. Use one
of the provided `.aset` Artisan settings in the "Artisan Config" folder, to connect to the roaster. Remember to select the correct port in Artisan!

# Wifi setup
At first start, Trident creates an access point with the SSID "Trident". Once you connect to it, you can use the
provided artisan config (in this folder) and it should connect automatically.

If you wish to setup Trident to connect to your home Wifi, you can set it up using a simple curl command or by visiting
the address in the browser:
```
curl http://trident.local/api/wifi?ssid=<your-wifi>&pass=<your-pass>
```

This will save the wifi ssid and credentials to Trident persistent storage. Now you can reboot/reset Trident and it will
connect to your Wifi automatically. 
If Trident can't find the specified Wifi or the password is incorrect, it will create the previously mentioned access
point, so that you can change the wifi data again.


# Simultaneous operation
While not entirely sensible, simultaneous operation is theoretically possible with Trident. You could for example
observe and control the roast, both from HiBean and Artisan at the same time.


# TODO list:

- PID control
- LED status signalling
- code cleanup and potential improvements

