# Installation and Updating

These steps are for VEXcode. Using PROS? See [Using PROS](pros.md).

## Install

1. **Download and install [Visual Studio Code](https://code.visualstudio.com/)**
2. **Install the VEX Robotics Extension**
    - Open the Extensions view (++ctrl+shift+x++, or ++cmd+shift+x++ on a Mac)
    - Search for and install **"VEX Robotics Extension"** by VEX Robotics
3. **Set up the project folder**
    - Download the latest **`RW-Template.zip`** from [Releases](https://github.com/richardbwang/RW-Template/releases) (not the "Source code" files)
    - Unzip it
    - Open the resulting folder in VS Code

Then follow the [Quick Start](quick-start.md).

## Update from an older version

Open your project in VS Code, then:

1. In the menu bar, click **Terminal**. On a Mac the menu bar is at the very top of the screen; on Windows and Linux it's at the top of the VS Code window (if you only see a ☰ icon there, click it first)
2. Click **Run Task...**
3. Pick **Update RW-Template** from the list

When it's done, build and download as usual.

!!! tip "Can't find the menu?"
    Press ++cmd+shift+p++ (++ctrl+shift+p++ on Windows), type `Run Task`, press ++enter++, and pick **Update RW-Template**.

Projects from 4.0.0 and older don't have this task yet. [Update them by hand once](#projects-from-400-and-older), then use the task after that.

The updater downloads the newest release and replaces RW's own files. It leaves your `custom` folder, your VS Code settings (apart from the update task itself) and any files you added alone. If you changed one of RW's files (say `src/motor-control.cpp`), it saves your copy in `rw-backup/` first and tells you which files it saved.

New settings have defaults, so your `custom` folder keeps building. The updater lists the settings that are new in the version you got, so you can copy any you want to change into `custom/src/robot-config.cpp`. The [release notes](https://github.com/richardbwang/RW-Template/releases) cover everything else that's new.

??? note "Updating without internet, or from the terminal"
    The task runs `tools/update.ps1` on Windows and `tools/update.sh` on Mac and Linux, and you can run them yourself in VS Code's terminal. To update a laptop without internet, download `RW-Template.zip` on another computer, copy it over, and give the updater its path:

    === "Windows"

        ```
        powershell -ExecutionPolicy Bypass -File tools\update.ps1 path\to\RW-Template.zip
        ```

    === "Mac and Linux"

        ```
        sh tools/update.sh path/to/RW-Template.zip
        ```

    If Windows says running scripts is disabled, the laptop's settings block PowerShell scripts. Update by hand instead (below).

### Projects from 4.0.0 and older

These don't have the updater yet, so update them by hand once:

1. Download and unzip the new `RW-Template.zip` as above, into a new folder. Don't unzip it over your old project: `src/main.cpp` is now `src/vexcode/main.cpp`, and an old copy left behind breaks the build
2. Copy your old project's **`custom`** folder into the new project, replacing the one there
3. If the four `controller_1.Axis` lines in `runDriver()` in `custom/src/user.cpp` end in `.value()`, change them to `.position()`. `.value()` goes up to 127 instead of 100, so the robot reached full speed with the stick only about 80% of the way over
4. Build. New settings have defaults, so an older `custom` folder keeps building. Coming from 2.x? Also copy the four distance reset sensors (`front_sensor`, `left_sensor`, `right_sensor`, `back_sensor`) from the new `custom/src/robot-config.cpp` and `custom/include/robot-config.h` into yours, with any free ports if your robot doesn't have them
5. To use new features, copy the new parts of the template's `custom/src/user.cpp` and `custom/src/robot-config.cpp` into yours

!!! warning
    Updating replaces anything you change outside the `custom` folder. The updater saves your copies in `rw-backup/`, but updating by hand doesn't, so copy out anything you want to keep.
