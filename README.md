## Soehnle Shape Sense Control 100 to MQTT Gateway

I bought a Soehnle Shape Sense Control 100 smart scale and wanted to get the data from the scale, so I can plot the values with Grafana.

Luckily found the GATT descriptors on the OpenScale project at: https://github.com/oliexdev/openScale/blob/master/android_app/app/src/main/java/com/health/openscale/core/bluetooth/scales/SoehnleHandler.kt

## Workflow:

- sync module time via NTP
- post the boot time to mqtt
- start BLE scan and look for the SCALE_DEVICE_NAME (Shape100)
- if a user steps on the scale, the scale activates bluetooth and the scan finds the device
- it then waits 15 seconds (REQUEST_DELAY_MS, the time it takes for the scale to weight and measure the impedance values for fat/water/muscle)
- then the module requests the measurement history from the scale (asynchron)
- it gives the scale 5 seconds (COLLECT_DELAY_MS) to send the history values
- the value with the highest time is selected and published to scale/measurement
- the user is determined by checking the weight limits in users.h
- additional the measure time + battery level is published
- the module then waits 40 (BT_DISCONNECT_DELAY) seconds (the time it takes for the scale to disable bluetooth) before going back to scan mode
- rinse/repeat

*I've ordered a NRF52840 sniffer dongle, maybe I can get the direct measurement in the future, without having to fetch the whole history first.*
