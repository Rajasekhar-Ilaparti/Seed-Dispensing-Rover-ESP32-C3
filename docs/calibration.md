# 🎯 First-time setup and calibration

Do these steps in order after the first upload. Every value is saved on the rover and survives power-off.

## 1. Motor direction

1. Lift the rover so the wheels are off the ground.
2. On the **Drive** tab, push the joystick gently forward. Both sides should spin forward.
3. If one side spins backwards, swap that motor pair's wires, or change `RIGHT_REVERSED` in the firmware.
4. If left and right steering are swapped, change `JOY_SWAP_LR`.

## 2. Drive power (low-speed driving)

1. On the **Drive** tab, set the speed slider to **20 %** and drive forward on the floor.
2. If the rover barely moves, open the **Calibration** tab. Raise **Minimum power** by +5 and press **Save drive power**. It applies straight away, so test again.
3. Repeat until it rolls smoothly at 20 %. Default: 110.

Starts are always soft (from PWM 60, building up at 300 PWM/s). If a side still won't start on the ground but spins in the air, check that its gearbox turns freely by hand. A stiff gearbox is enough to stop it.

## 2b. Gyro

1. Switch on with the rover sitting still and level.
2. On the **Calibration** tab, the gyro card should show **Online**.
3. Press **Test right turn** once. The rover learns which gyro direction is "right" and saves it. The System tab then shows *Turn direction: Learned*.
4. If the heading creeps while the rover sits still, press **Recalibrate gyro** with the rover still.

## 3. 90° turns

| Setting | Meaning | Start with |
|---|---|---|
| Gyro turns | Stop the turn on the measured angle | On |
| Turn angle | Target angle per turn | 90° |
| Stop early | Stop this many degrees before the target, to allow for coasting | 8° |
| Right-turn time | Used only when the gyro is off or missing | 480 ms |

1. Press **Test right turn**. The status line then shows the real angle, for example *Turned 91.5 deg (gyro)*.
2. Overshoots: increase **Stop early**. Undershoots: decrease it.
3. Press **Save settings**.

## 4. Square path

1. Set **Forward time per side**, for example 1000 ms.
2. Press **Run square**. The rover drives 4 × (forward, then turn right) and should end close to where it started.
3. With **Hold straight** on, the gyro corrects drift on the straight legs.

## 5. Seed hopper (Dispenser tab)

The hopper angles are **locked in the firmware** at closed **135°** and open **75°**. They're calibrated for this dispenser mechanism, and the controller can't change them. A wrong angle can jam the hopper or spill seeds.

If the mechanism is ever rebuilt, change `servoClosed` and `servoOpen` near the top of `C3_Seed_Rover.ino`, test carefully, and upload again.

On the Dispenser tab you can tune the timing only:

1. Set **Swing speed** (°/s, 0 = full speed), **Hold open** (ms) and **Swings per dispense**.
2. Press **Test** with some seeds in the hopper. Test uses the values on screen without saving them.
3. Press **Save** and confirm. The Drive tab uses the new values straight away.

Firmware defaults: 200 °/s, 100 ms hold, 1 swing.

## 6. Continuous dispense

On the **Drive** tab, open **Dispense options** and set the interval between drops (100–10000 ms). These options are saved on the phone. Turn on **Continuous dispense** and drive. The spacing between seeds is roughly *drive speed × interval*.
