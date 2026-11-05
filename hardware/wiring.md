# 🔌 Wiring

All connections for the Seed Dispensing Rover. The pin numbers match the firmware. If you change any, change the `#define`s at the top of `C3_Seed_Rover.ino` too.

## Block diagram

```text
 2S Li-ion ──BMS──Switch──Fuse──┬──────────────► DRV8833 VM ──► AO1/AO2: left motors (2 in parallel)
 (7.4 V nom, 8.4 V full)        │                            └► BO1/BO2: right motors (2 in parallel)
                                │
                                └──► Buck IN ──5.0 V──┬──► ESP32-C3 5V pin
                                                      └──► Servo + (red)

 ESP32-C3 3V3 ──► MPU6050 VCC

 GND: battery −, DRV8833 GND, buck IN− / OUT−, ESP32-C3 GND, servo −, MPU6050 GND  (all common)
```

## 1. Power

| From | To | Notes |
|---|---|---|
| 2S battery **+** (through the BMS) | Switch → fuse → **DRV8833 VM** and **buck IN+** | 3–5 A inline fuse recommended |
| 2S battery **−** (BMS P−) | **DRV8833 GND** and **buck IN−** | Use a thick wire. Connect the buck's IN− at the same point |
| Buck **OUT+** (set to **5.0 V**) | **ESP32-C3 5V** pin and **servo red (+)** | Never connect the buck output to the 3V3 pin |
| Buck **OUT−** | **ESP32-C3 GND** and **servo brown/black (−)** | |
| ESP32-C3 **3V3** | **MPU6050 VCC** | The board's own regulator output |

> ⚠️ **Set the buck to 5.0 V with a multimeter before connecting anything to it.**

> ⚠️ **Battery straight to VM:** a full 2S pack is 8.4 V, which is above the usual 6 V rating of BO motors. The firmware caps joystick output at PWM 200 of 255 (about 6.6 V average at full charge). Running near 100 % for long periods heats the motors and the DRV8833.

> ⚠️ **BMS current rating:** four BO motors starting together can draw 3–5 A for a moment. A BMS with a low over-current limit, or a flat battery, cuts the power and the ESP32 restarts. Use a 2S BMS rated **8–10 A** or more.

## 2. ESP32-C3 Super Mini

| GPIO | Connects to | Function |
|---|---|---|
| 3 | DRV8833 **AIN1** | Left motors |
| 4 | DRV8833 **AIN2** | Left motors |
| 5 | DRV8833 **BIN1** | Right motors |
| 6 | DRV8833 **BIN2** | Right motors |
| 7 | DRV8833 **STBY** (EEP / nSLEEP) | Driver enable |
| 10 | Servo **signal** (orange/yellow) | Seed hopper |
| 20 | MPU6050 **SDA** | I2C data |
| 21 | MPU6050 **SCL** | I2C clock |
| 3V3 | MPU6050 VCC | Sensor power |
| 5V | Buck OUT+ | Board power |
| GND | Common ground | |

Leave **GPIO 2, 8 and 9** unconnected, because they're boot strapping pins. **GPIO 0 and 1** are free, for example for battery-voltage sensing.

## 3. Motors (DRV8833 outputs)

| DRV8833 | Motors |
|---|---|
| AO1 / AO2 | Both **left** motors, in parallel |
| BO1 / BO2 | Both **right** motors, in parallel |

The right side is reversed in software (`RIGHT_REVERSED true`). If one side drives backwards, swap that motor pair's two wires or change the flag.

## 4. Servo (seed hopper)

| Servo wire | Connect to |
|---|---|
| Orange / yellow (signal) | GPIO 10 |
| Red (+) | Buck 5 V (**not** the ESP32's 3V3) |
| Brown / black (−) | Common GND |

## 5. MPU6050 (GY-521)

| MPU6050 pin | Connect to |
|---|---|
| VCC | ESP32-C3 3V3 |
| GND | GND |
| SDA | GPIO 20 |
| SCL | GPIO 21 |
| AD0 | Leave unconnected (I2C address 0x68) |
| XDA, XCL, INT | Leave unconnected |

**Mounting:** flat, chip side up, near the centre of the chassis, on foam tape and away from the motors. Keep the SDA and SCL wires short (under 20 cm).

## 6. Capacitors

| Where | Value | Why |
|---|---|---|
| DRV8833 VM to GND, close to the driver | 470–1000 µF, 16 V or more | Supplies the motor starting surge |
| Servo 5 V to GND, near the servo | 470 µF, 10 V or more | Servo current spikes |
| Across each motor's terminals | 100 nF ceramic (optional) | Reduces electrical noise |

## 7. Checks before switching on

1. **USB unplugged** while the battery is connected. Use OTA for uploads.
2. Buck output measured at **5.0 V**.
3. All grounds common.
4. Battery charged. At rest, a 2S pack should read above about 7.0 V.
5. At power-on, keep the rover **still for about 2 s** while the gyro calibrates, and leave the servo arm room to move to its closed angle.
