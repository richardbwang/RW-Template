# Driver Control

Edit the `runDriver()` function in `custom/src/user.cpp`.

By default, it uses tank drive:

```cpp
driverTank(ch3, ch2);
```

Other options:

- Arcade on the left stick: `driverArcade(ch3, ch4);`
- Split arcade: `driverArcade(ch3, ch1);`
- X-drive, mecanum or H-drive: `driverHolonomic(ch3, ch4, ch1);`, or `driverFieldCentric(ch3, ch4, ch1);` to drive relative to the field ([Drive types](drive-types.md#driver-control))
- Add button controls for your mechanisms

`driver_curve` (0 = linear, 2-6 is typical) softens small stick movements without lowering top speed. `driver_deadband` ignores tiny stick drift.
