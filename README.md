## Soehnle Shape Sense Control 100 --BLE--> ESP32c3 --WIFI--> MQTT

I bought a Soehnle Shape Sense Control 100 smart scale and wanted to get the data directly from the scale for plotting the values with Grafana.

The scale supports the standard body composition GATT service at "0000181b-0000-1000-8000-00805f9b34fb".
Measurement indications can be aquired with "00002a9c-0000-1000-8000-00805f9b34fb".

## Workflow:

- sync module time via NTP
- post the boot time to mqtt
- start BLE scan and look for the SCALE_DEVICE_NAME (Shape100)
- if a user steps on the scale, the scale activates bluetooth and the scan finds the device
- a body composition measurement indication is invoked
- wait for the indication callback or timeout
- publish the measurement data
- additional the measure time + battery level is published
- the module then waits 40 (BT_DISCONNECT_DELAY) seconds (the time it takes for the scale to disable bluetooth) before going back to scan mode
- rinse/repeat
