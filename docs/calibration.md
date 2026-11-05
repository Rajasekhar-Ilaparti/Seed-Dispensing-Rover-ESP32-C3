# 🎯 First-time setup and calibration

Do these steps in order after the first upload. Every value is saved on the rover and survives power-off.

## 1. Motor direction

1. Lift the rover so the wheels are off the ground.
2. On the **Drive** tab, push the joystick gently forward. Both sides should spin forward.
3. If one side spins backwards, swap that motor pair's wires, or change `RIGHT_REVERSED` in the firmware.
4. If left and right steering are swapped, change `JOY_SWAP_LR`.

## 2. Gyro

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

1. Drag the **servo position** slider until the hopper is fully closed. Press **Set as closed**.
2. Move it until seeds fall freely. Press **Set as open**.
3. Set **Swing speed** (°/s, 0 = full speed), **Hold open** (ms) and **Swings per dispense**.
4. Press **Test** with some seeds in the hopper, adjust, then press **Save**.

Firmware defaults: closed 135°, open 75°, 200 °/s, 100 ms hold, 1 swing.

## 6. Continuous dispense

On the **Drive** tab, open **Dispense options** and set the interval between drops (100–10000 ms). These options are saved on the phone. Turn on **Continuous dispense** and drive. The spacing between seeds is roughly *drive speed × interval*.
