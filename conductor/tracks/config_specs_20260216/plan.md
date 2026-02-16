# Plan: Configuring project with robot specifications

## Phase 1: Hardware Port Mapping
- [x] Task: Configure motor ports for drivetrain and scoring mechanisms in `custom/src/robot-config.cpp`. 810648b
    - [ ] Map left chassis motor group.
    - [ ] Map right chassis motor group.
    - [ ] Map scoring mechanism motors (intake, lift, etc.).
- [ ] Task: Configure sensor ports for autonomous navigation.
    - [ ] Assign inertial sensor port.
    - [ ] Assign odometry tracking wheel ports (if used).
    - [ ] Assign distance sensor ports for resets.
- [ ] Task: Conductor - User Manual Verification 'Hardware Port Mapping' (Protocol in workflow.md)

## Phase 2: Physical Constants & PID Tuning
- [ ] Task: Set chassis physical constants in `custom/src/robot-config.cpp`.
    - [ ] Define `distance_between_wheels`.
    - [ ] Calculate and set `wheel_distance_in` based on gear ratio and wheel diameter.
- [ ] Task: Set initial PID constants for motion control.
    - [ ] Configure `distance_kp`, `distance_ki`, `distance_kd`.
    - [ ] Configure `turn_kp`, `turn_ki`, `turn_kd`.
    - [ ] Configure `heading_correction_kp`, `heading_correction_ki`, `heading_correction_kd`.
- [ ] Task: Conductor - User Manual Verification 'Physical Constants & PID Tuning' (Protocol in workflow.md)
