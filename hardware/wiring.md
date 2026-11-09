# 🔌 Wiring

All connections for the Seed Dispensing Rover. The pin numbers match the firmware. If you change any, change the `#define`s at the top of `C3_Seed_Rover.ino` too.

The rover now runs on a **TC1508A** dual motor driver module. It was first built with a **DRV8833**; that wiring still works with three flag changes (see [Using a DRV8833 instead](#using-a-drv8833-instead)).

## Block diagram

```text
 2S Li-ion ──BMS──Switch──Fuse──┬──────────────► TC1508A + ──► MOTOR-A: right motors (2 in parallel)
 (7.4 V nom, 8.4 V full)        │                           └► MOTOR-B: left motors  (2 in parallel)
                                │
                                └──► Buck IN ──5.0 V──┬──► ESP32-C3 5V pin
                                                      └──► Servo + (red)

 ESP32-C3 3V3 ──► MPU6050 VCC

 GND: battery −, TC1508A −, buck IN− / OUT−, ESP32-C3 GND, servo −, MPU6050 GND  (all common)
```

## 1. Power

| From | To | Notes |
|---|---|---|
| 2S battery **+** (through the BMS) | Switch → fuse → **TC1508A +** and **buck IN+** | 3–5 A inline fuse recommended |
| 2S battery **−** (BMS P−) | **TC1508A −** and **buck IN−** | Use a thick wire. Connect the buck's IN− at the same point |
| Buck **OUT+** (set to **5.0 V**) | **ESP32-C3 5V** pin and **servo red (+)** | Never connect the buck output to the 3V3 pin |
| Buck **OUT−** | **ESP32-C3 GND** and **servo brown/black (−)** | |
| ESP32-C3 **3V3** | **MPU6050 VCC** | The board's own regulator output |

> ⚠️ **Set the buck to 5.0 V with a multimeter before connecting anything to it.**

> ⚠️ **TC1508A supply voltage:** the TC1508A chip's datasheet gives **7.2 V absolute maximum** (2.4–7.2 V recommended), although module listings often say 2–10 V. A 2S pack (7.4 V nominal, 8.4 V full) is **above the chip's rating**. This rover runs it straight from the pack anyway, as a deliberate risk. The safe option is a buck converter set to about 6 V (3 A or more) for the driver. Don't run it from a freshly charged pack, and fit a heatsink on top of the chip, kept clear of the pins.

> ⚠️ **Heat:** the TC1508A has thermal shutdown but no current limit, and it gets very hot at high PWM. The firmware caps joystick output at **PWM 100 of 255** (`JOY_MAX_PWM`) for this reason.

> ⚠️ **BMS current rating:** four BO motors starting together can draw 3–5 A for a moment. A BMS with a low over-current limit, or a flat battery, cuts the power and the ESP32 restarts. Use a 2S BMS rated **8–10 A** or more.

## 2. ESP32-C3 Super Mini

| GPIO | Connects to | Function |
|---|---|---|
| 3 | TC1508A **INT1** | MOTOR-A (right motors) |
| 4 | TC1508A **INT2** | MOTOR-A (right motors) |
| 5 | TC1508A **INT3** | MOTOR-B (left motors) |
| 6 | TC1508A **INT4** | MOTOR-B (left motors) |
| 7 | Not connected | STBY on a DRV8833; the TC1508A has no enable pin |
| 10 | Servo **signal** (orange/yellow) | Seed hopper |
| 20 | MPU6050 **SDA** | I2C data |
| 21 | MPU6050 **SCL** | I2C clock |
| 3V3 | MPU6050 VCC | Sensor power |
| 5V | Buck OUT+ | Board power |
| GND | Common ground | |

Leave **GPIO 2, 8 and 9** unconnected, because they're boot strapping pins. **GPIO 0 and 1** are free, for example for battery-voltage sensing.

## 3. Motors (TC1508A outputs)

| TC1508A | Motors |
|---|---|
| MOTOR-A | Both **right** motors, in parallel |
| MOTOR-B | Both **left** motors, in parallel |

The firmware flags for this wiring are `SIDES_SWAPPED true`, `RIGHT_REVERSED false` and `JOY_SWAP_LR false`. If one side drives backwards, swap that motor pair's two wires. If the whole rover steers the wrong way, see [docs/calibration.md](../docs/calibration.md).

### Using a DRV8833 instead

Wire GPIO 3/4 → AIN1/AIN2 (left motors on AO1/AO2), GPIO 5/6 → BIN1/BIN2 (right motors on BO1/BO2) and GPIO 7 → STBY. Then set `SIDES_SWAPPED false`, `RIGHT_REVERSED true` and `JOY_SWAP_LR true`. The DRV8833 cuts out on sudden current surges, so it needs a higher `JOY_MIN_PWM` (about 110) and allows a higher `JOY_MAX_PWM` (up to 200).

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
| Motor driver + to −, close to the driver | 470–1000 µF, 16 V or more | Supplies the motor starting surge (the TC1508A module has 2 × 100 µF on board) |
| Servo 5 V to GND, near the servo | 470 µF, 10 V or more | Servo current spikes |
| Across each motor's terminals | 100 nF ceramic (optional) | Reduces electrical noise |

## 7. Checks before switching on

1. **USB unplugged** while the battery is connected. Use OTA for uploads.
2. Buck output measured at **5.0 V**.
3. All grounds common.
4. Battery charged. At rest, a 2S pack should read above about 7.0 V.
5. At power-on, keep the rover **still for about 2 s** while the gyro calibrates, and leave the servo arm room to move to its closed angle.
