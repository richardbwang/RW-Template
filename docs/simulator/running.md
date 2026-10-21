# Running the Simulator

Run your autons on a computer and watch a replay, no robot needed. The simulator runs your real RW code with a simulated robot, sensors, and Override field, and writes a replay page showing where the robot really went next to where RW thought it was.

<div class="rw-replay" data-src="../../replays/Path-Example.html" data-title="Path Example"></div>

## One-time setup: install a C++ compiler

=== "macOS"

    Open Terminal and run:

    ```bash
    xcode-select --install
    ```

=== "Windows"

    The simulator runs in WSL (Linux inside Windows). Some school laptops block WSL.

    1. Open PowerShell as administrator, run this, and restart:

        ```
        wsl --install
        ```

    2. Finish the Ubuntu setup that opens
    3. In VS Code's terminal, type `wsl`. The terminal is now Linux, still in your project folder
    4. Run:

    ```bash
    sudo apt update && sudo apt install -y g++
    ```

=== "Linux"

    Install `g++`, e.g.:

    ```bash
    sudo apt install g++
    ```

## Run an auton

From the project folder (on Windows, type `wsl` in VS Code's terminal first):

```bash
sh sim/run-sim.sh "Async Example"
```

Use the name you gave `addAuton()`. To list them:

```bash
sh sim/run-sim.sh --list
```

It builds your current code (everything in `src/` and `custom/`, minus `src/vexcode/main.cpp`), runs `runPreAutonomous()` and then your auton on a simulated robot, and writes a replay such as `sim/out/Async-Example.html`. In a PROS project it builds your `src/` files and runs `initialize()` and `autonomous()` instead (see [Using PROS](../getting-started/pros.md#simulator)).

Open the replay in any browser to play it back. In VS Code, right-click it and choose **Reveal in Finder** or **Reveal in File Explorer**, then double-click it. It shows the field, the path the robot really drove, the path RW thought it drove, and the error between them. The terminal prints the same summary.

## Options

| Option | What it does |
| --- | --- |
| `--list` | List the autons added with `addAuton()` |
| `--start X,Y,HEADING` | Where the robot starts, in field-centered inches and compass degrees (default `-35,-60,0`). Use the same spot your auton assumes |
| `--time SECONDS` | Stop after this long (default 15; use 60 for skills) |
| `--ideal` | No wheel slip, IMU error, or sensor noise |
| `--walls` | Empty field without Override's goals, loaders, and stacks |
| `--size INCHES`, `--seed N`, `--out FILE` | Field size, random seed, and where to write the replay |

## What it simulates

- **Robot:** your drive (tank, X-drive, mecanum or H-drive) with the measurements in `robot-config.cpp` (`rw-config.cpp` on PROS), with motor lag, wheel slip, turning scrub, and sideways skid. To match your robot exactly, [calibrate it](calibration.md)
- **Walls:** the robot stops at the perimeter but its wheels keep spinning, so odometry drifts like it does on a real field
- **Sensors:**
    - IMU with a small scale error and drift
    - tracking wheels
    - the four reset distance sensors, which update every 33 ms, lag about 40 ms, and see the walls, goals, loaders, and stacks
- **Code:** all of your code runs unchanged, including threads, `wait()`, MCL, and the auton selector

## Limitations

- The robot drives through goals and game elements; only the walls stop it
- Mechanisms just spin, controller inputs read as zero, and the Brain screen draws nothing
- The SD card is the folder `sim/out/sdcard`. The Drive Calibration auton saves its log there if you give it time: `--time 20`
- Distance sensors other than the four reset sensors read "nothing detected"

If your code uses a VEX or PROS function the simulator doesn't have yet, the build error names it. Add it to `sim/vex/v5_vcs.h` (`sim/pros/pros-sim.h` on PROS), or ask on [Discord](https://discord.gg/9FHyyFe22p).
