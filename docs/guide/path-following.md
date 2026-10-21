# Path Following

RW builds smooth paths through waypoints (centripetal Catmull-Rom splines), or takes a path drawn in path.jerryio. It plans the speed along the path, slowing down for curves, and follows it with one of two controllers:

- **Pure pursuit** chases a point ahead on the path. It's forgiving and follows the path's shape
- **RAMSETE** follows the path on a timetable, so timing is very repeatable

```cpp
setPose(-36, -60, 0);   // Start at the path's first point, facing along it
Path out = pathFromPoints({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}, {34, -24}});
followPath(out, 5000);                   // Pure pursuit
Path back = pathFromPoints({{34, -24}, {12, -30}, {-12, -22}, {-36, -32}, {-36, -52}});
followPathRamsete(back, 5000, true);     // RAMSETE, driving back along it backward
```

- `pathFromPoints({{x, y}, ...})` makes a smooth curve through each point. Add a third number to slow down there, e.g. `{0, -12, 0.5}` for half speed
- Paths use the same positions as odometry, so start the auton with `setPose()` at the path's first point
- `followPath(path, time_limit, reverse, exit, max_speed, lookahead)`: pure pursuit. A bigger `path_lookahead` is smoother but cuts corners more
- `followPathRamsete(path, time_limit, reverse, exit, max_speed)`: RAMSETE keeps to a timetable, so it's the most repeatable
- Both end lined up with the path's last direction, usually within 1-2" of the end point and a few degrees of its heading. If the next step needs it exact, follow with `moveToPoint()`, `turnToAngle()`, or a distance reset
- Both work with `runAsync()`, the wait helpers, and `cancelMotion()`

## In the simulator

This is the code above (the template's **Path Example** auton):

<div class="rw-replay" data-src="../../replays/Path-Example.html" data-title="Path Example"></div>

## Drawing paths in path.jerryio

[path.jerryio](https://path.jerryio.com) lets you draw paths on top of the field.

1. Set the format to **LemLib v0.5** (the text format) and pick the **V5RC 2027 - Override** field
2. Draw the path. Speeds you set in jerryio cap the speed along that part of the path. RW plans its own slowing for curves and for the stop at the end, from your robot's settings
3. Save the path file and open it in VS Code. The first line is the start point (x, y, speed)
4. Paste the whole file into your auton:

```cpp
setPose(-36, -60, 0);   // The file's first point, with the heading the robot starts at
Path drawn = pathFromJerryIO(R"(
-36.000, -60.000, 100.000
-36.000, -58.000, 100.000
...the rest of the file...
)");
followPath(drawn, 5000);
```
