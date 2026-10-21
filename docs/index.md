---
hide:
  - navigation
  - toc
---

<div class="rwx-hero" markdown>
<div class="rwx-hero__text" markdown>

<p class="rwx-kicker">VEX V5 · Autonomous template</p>

<h1 class="rwx-title">Write an auton.<br>Watch it drive.<br><span class="rwx-glow">Then run it for real.</span></h1>

Odometry, PID, path following, MCL and a simulator, with or without tracking wheels, for VEXcode or PROS. Built by Richard Wang of team 1698V.

<div class="rwx-actions" markdown>

[Get started](getting-started/installation.md){ .md-button .md-button--primary }
[Try the PID Playground](guide/pid-playground.md){ .md-button }
[:fontawesome-brands-discord: Join the Discord](https://discord.gg/9FHyyFe22p){ .md-button .rwx-discord }

</div>

<a class="rwx-stat" href="https://github.com/richardbwang/RW-Template/releases"><b data-downloads>1,000+</b><span>downloads</span></a>

</div>
<div class="rwx-hero__art">
<svg class="rwx-drive" viewBox="0 0 512 512" role="img" aria-label="A robot drives a path that draws the RW logo">
  <defs>
    <linearGradient id="rwx-chrome" gradientUnits="userSpaceOnUse" x1="80" y1="420" x2="480" y2="90">
      <stop offset="0" stop-color="#8b9099"/><stop offset="0.6" stop-color="#d9dadd"/><stop offset="1" stop-color="#ffffff"/>
    </linearGradient>
    <filter id="rwx-glow" x="-30%" y="-30%" width="160%" height="160%">
      <feGaussianBlur stdDeviation="8" result="b"/>
      <feComponentTransfer in="b" result="s"><feFuncA type="linear" slope="0.3"/></feComponentTransfer>
      <feMerge><feMergeNode in="s"/><feMergeNode in="SourceGraphic"/></feMerge>
    </filter>
  </defs>
  <g class="rwx-tiles">
    <path d="M85.3 0V512M170.7 0V512M256 0V512M341.3 0V512M426.7 0V512M0 85.3H512M0 170.7H512M0 256H512M0 341.3H512M0 426.7H512"/>
  </g>
  <g transform="translate(256 256) scale(0.92) translate(-256 -256)">
    <g transform="translate(-14 10)">
      <path class="rwx-plan" pathLength="1" d="M96 404 C96 300 100 180 108 128 C112 100 150 100 190 104 C250 110 262 200 200 236 C168 250 136 252 112 256 C160 272 210 330 238 392 C246 410 262 410 270 390 C282 330 296 214 312 206 C330 198 338 320 350 380 C358 410 376 410 386 390 C404 340 424 230 444 140"/>
      <path class="rwx-ink" pathLength="1" filter="url(#rwx-glow)" stroke="url(#rwx-chrome)" d="M96 404 C96 300 100 180 108 128 C112 100 150 100 190 104 C250 110 262 200 200 236 C168 250 136 252 112 256 C160 272 210 330 238 392 C246 410 262 410 270 390 C282 330 296 214 312 206 C330 198 338 320 350 380 C358 410 376 410 386 390 C404 340 424 230 444 140"/>
      <path class="rwx-arrow" d="M456.2 85.3 L479.1 147.8 L408.9 132.2 Z"/>
      <circle class="rwx-start" cx="96" cy="404" r="15"/>
      <g class="rwx-bot" transform="translate(96 404)">
        <rect x="-24" y="-24" width="48" height="48" rx="9"/>
        <path d="M0 -36 L9 -22 L-9 -22 Z"/>
      </g>
    </g>
  </g>
</svg>
<div class="rwx-readout" aria-hidden="true"><span>x <b data-k="x">-46.0</b></span><span>y <b data-k="y">-52.3</b></span><span>θ <b data-k="h">0°</b></span></div>
<button type="button" class="rwx-replay-btn" hidden>▶ Drive it again</button>
</div>
</div>

## How it works

<div class="rwx-steps" markdown>

<div class="rwx-step" markdown>
<span class="rwx-step__n">1</span>

**Set up your robot**

Ports and wheel measurements in one file.

[Configuration](guide/configuration.md)
</div>

<div class="rwx-step" markdown>
<span class="rwx-step__n">2</span>

**Tune it**

Run the Tuning Test, or get a feel for it in the playground first.

[PID Playground](guide/pid-playground.md)
</div>

<div class="rwx-step" markdown>
<span class="rwx-step__n">3</span>

**Write autons**

Drives, turns, curves, paths, and mechanisms timed to the drive.

[Which motion?](guide/autonomous.md#which-motion-should-i-use)
</div>

<div class="rwx-step" markdown>
<span class="rwx-step__n">4</span>

**Test on your computer**

Watch a replay in the simulator before you touch the field.

[Simulator](simulator/running.md)
</div>

</div>

## See it drive

The template's [Path Example](guide/path-following.md) in the [simulator](simulator/running.md): an S-curve with pure pursuit, then back along it with RAMSETE. **Blue** is where the robot really went, **orange** is where RW thought it was. Press **Play**.

<div class="rw-replay" data-src="replays/Path-Example.html" data-title="Path Example"></div>

## Everything in the box

<div class="grid cards rwx-cards" markdown>

-   :material-map-marker-path:{ .lg .middle } __Odometry, with or without tracking wheels__

    ---

    Tracks the robot's position from drive encoders and/or tracking wheels plus the IMU. All four setups share one tested implementation.

    [:octicons-arrow-right-24: Configuration](guide/configuration.md)

-   :material-vector-curve:{ .lg .middle } __Motions for every job__

    ---

    Drive, turn, swing, arc, move to point, boomerang to a pose, and chain them together without stopping.

    [:octicons-arrow-right-24: Which motion should I use?](guide/autonomous.md#which-motion-should-i-use)

-   :material-timer-sand:{ .lg .middle } __Async motions__

    ---

    Run a motion in the background and time mechanisms to it with `waitUntilTraveled()`, `waitUntilNear()` and `cancelMotion()`.

    [:octicons-arrow-right-24: Async motions](guide/async-motions.md)

-   :material-chart-bell-curve-cumulative:{ .lg .middle } __Motion profiling__

    ---

    `driveProfiled()` and `turnProfiled()` speed up, cruise and slow down smoothly using a feedforward model of your drive.

    [:octicons-arrow-right-24: Motion profiles](guide/motion-profiles.md)

-   :material-road-variant:{ .lg .middle } __Path following__

    ---

    Smooth paths through waypoints or drawn in path.jerryio, followed with pure pursuit or RAMSETE.

    [:octicons-arrow-right-24: Path following](guide/path-following.md)

-   :material-satellite-variant:{ .lg .middle } __Distance resets and MCL__

    ---

    Reset position off the walls, or let Monte Carlo Localization keep correcting odometry while you drive.

    [:octicons-arrow-right-24: Localization](guide/localization.md)

-   :material-monitor-dashboard:{ .lg .middle } __Simulator__

    ---

    Run your real autons on a computer and watch a replay on an Override field. No robot needed.

    [:octicons-arrow-right-24: Simulator](simulator/running.md)

-   :material-arrow-all:{ .lg .middle } __Any drivetrain__

    ---

    Tank, X-drive, mecanum or H-drive. Strafe to a point while turning, and drive field-centric in driver control.

    [:octicons-arrow-right-24: Drive types](guide/drive-types.md)

-   :material-swap-horizontal:{ .lg .middle } __VEXcode or PROS__

    ---

    The same motions, settings and simulator either way. On PROS, two commands add it to your project.

    [:octicons-arrow-right-24: Using PROS](getting-started/pros.md)

-   :material-folder-lock-outline:{ .lg .middle } __Easy updates__

    ---

    Everything you change lives in `custom/`. One VS Code task updates the rest to the newest version.

    [:octicons-arrow-right-24: Updating](getting-started/installation.md)

</div>

Also included: a universal PID class, an auton selector on the Brain and controller, driver curves, IMU scale correction, and a stall exit for stuck motions.

<div class="rwx-cta" markdown>

### Stuck? Come say hi.

Ask on the Discord, where other teams using RW hang out too, or [open an issue](https://github.com/richardbwang/RW-Template/issues). Follow team 1698V on [YouTube](https://www.youtube.com/@1698V).

[:fontawesome-brands-discord: Join the Discord](https://discord.gg/9FHyyFe22p){ .md-button .md-button--primary .rwx-discord }

</div>
