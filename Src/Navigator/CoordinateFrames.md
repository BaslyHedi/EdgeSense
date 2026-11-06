# EdgeSense Coordinate Frames & Sensor Axis Convention

## Hardware: Raspberry Pi 5 + Sense HAT v2 (LSM9DS1)

### Physical Axis Layout

```
Sense HAT v2 — viewed from above, LED matrix facing up, GPIO connector at bottom

          ← +Y (away from GPIO) →
           ________________________
  +X      |                       |
(toward → |      LED Matrix       |  ← joystick on +X edge
joystick) |_______________________|
               GPIO connector

  +Z = UP (perpendicular to the PCB surface, out of the board)
```

---

## Euler Angle Convention (ZYX — AhrsEngine)

Angles are extracted from the quaternion using the ZYX convention
(Yaw applied first, then Pitch, then Roll about the final body X axis).
All angles output in degrees, range [−180, +180].

| Angle     | Axis        | Positive physical direction      | Accel signature at ±90° |
|-----------|-------------|----------------------------------|-------------------------|
| Roll (+φ) | X (joystick)| GPIO side goes **DOWN**          | ay → +9.81 m/s²         |
| Roll (−φ) | X (joystick)| Joystick side goes **DOWN**      | ay → −9.81 m/s²         |
| Pitch (+θ)| Y (away-GPIO)| Joystick side goes **DOWN**     | ax → −9.81 m/s²         |
| Pitch (−θ)| Y (away-GPIO)| Away-GPIO side goes **DOWN**    | ax → +9.81 m/s²         |
| Yaw (+ψ)  | Z (up)      | CCW rotation from above          | magnetometer-driven     |

### Gravity Vector Convention

The accelerometer measures **specific force** (the reaction force that opposes gravity).
At rest on a flat surface, the sensor reads az = +9.81 m/s².
The Madgwick gravity objective function assumes the body-frame gravity reference is
`[0, 0, +1]` (normalised), consistent with the above.

---

## Gyro Axis Sign Corrections

The LSM9DS1 gyro and accelerometer share the same physical coordinate frame per the
datasheet. However, an empirical test (Roll to +90° on the Sense HAT v2 mounting)
showed that the gyro X axis sign is inverted relative to the Madgwick quaternion
integration convention:

```
qDot.x = 0.5 * (q.w * gx + ...)
```

At identity quaternion: `qDot.x = 0.5 * gx`. For Roll to increase, gx must be positive.
The sensor reported gx < 0 during the rotation toward Roll = +90°. Fix: negate gx.

| Axis | Expected sign for positive angle | Observed in log       | Correction in AhrsEngine.cpp |
|------|----------------------------------|-----------------------|------------------------------|
| gx   | + when Roll increases            | negative (inverted)   | `gx *= -DEG_TO_RAD`          |
| gy   | + when Pitch increases           | negative (inverted)   | `gy *= -DEG_TO_RAD`          |
| gz   | + when Yaw increases             | positive (correct)    | `gz *= +DEG_TO_RAD`          |

### How to Verify an Axis Sign

1. Start the application with logging enabled.
2. Hold the board flat and stationary; confirm Roll ≈ Pitch ≈ 0°.
3. Make one slow, deliberate rotation in the **positive** direction for the axis under test
   (see the Euler table above for which physical direction is positive).
4. Read the raw `gx` / `gy` / `gz` field from the DATA log line.
5. If the raw value is **negative** while the angle should be increasing → that axis is
   inverted. Add a `-` prefix to its conversion factor in `AhrsEngine.cpp` and update
   the correction table above.

---

## File Reference

Corrections are applied in:
`Src/Navigator/AhrsEngine.cpp` — step 3 of `AhrsEngine::update()` (unit conversion block).
