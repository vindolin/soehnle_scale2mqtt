Workflow:

- sync module time via NTP
- post the boot time to mqtt
- start BLE scan and look for the SCALE_DEVICE_NAME (Shape100)
- if a user steps on the scale, the scale activates bluetooth and the scan finds the device
- it then waits 15 seconds (REQUEST_DELAY_MS, the time it takes for the scale to weight and measure the impedance values for fat/water/muscle)
- then the module requests the measurement history from the scale (asynchron)
- it gives the scale 5 seconds (COLLECT_DELAY_MS) to send the history values
- the value with the highest time is selected and published to scale/measurement
- additional the measure time + battery level is published
- the module then waits 40 (BT_DISCONNECT_DELAY) seconds (the time it takes for the scale to disable bluetooth) before going back to scan mode
- rinse/repeat
