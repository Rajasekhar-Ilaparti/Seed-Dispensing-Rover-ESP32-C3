# 🔧 Troubleshooting

The **System** tab shows the *Last reset* reason and an event log. Check it first.

| Symptom | Likely cause | Fix |
|---|---|---|
| Rover restarts and Wi-Fi disappears when the motors start. *Last reset* says **Power on** or **BROWNOUT** | Battery low, or the BMS cuts out on the starting current | Charge the battery. Use a 2S BMS rated 8–10 A. Add 470–1000 µF at the DRV8833 VM. Run thick ground wires |
| *Last reset* says **Crash** or **Watchdog** | Firmware problem | Note what you were doing and check the event log |
| Phone can't open 192.168.4.1 | Another device (often the PC) is on C3-ROVER, which allows only one | Disconnect the other device |
| Page shows **Offline** | Out of range, or the phone left the rover's Wi-Fi | Rejoin C3-ROVER. The page reconnects by itself |
| Old page appears after an update | Browser cache | Refresh the page |
| One side stalls on pivot turns | Fast-decay PWM or a weak supply | Keep `SLOW_DECAY true` and `PWM_FREQ 1000` |
| Turns are not 90° | Coasting, or the gyro direction is not learned yet | Run **Test right turn**, then adjust **Stop early** |
| Heading drifts while standing still | Moved during boot calibration | Press **Recalibrate gyro** with the rover still |
| *MPU6050 not detected* | Wiring | SDA → GPIO 20, SCL → GPIO 21, VCC → 3V3, GND. AD0 unconnected |
| Motors stop and the status shows **TILT** | Rover tilted past 35° | Set it level. Driving resumes after 1 s below 25° |
| USB upload fails with "Access denied" on the COM port | Serial Monitor is holding the port | Close Serial Monitor and retry |
| OTA upload can't find the rover | PC not on C3-ROVER, or the phone holds the only connection | Disconnect the phone and connect the PC to C3-ROVER |
