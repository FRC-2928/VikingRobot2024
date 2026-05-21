# VikingRobot2024 — Robot Configuration Reference

This document captures all robot-specific configuration, hardware, control schemes, and behavior for FRC Team 2928's 2024 competition robot. It is intended to serve as the authoritative reference for rebuilding equivalent functionality under the new goal-based Superstructure architecture.

---

## Team & Robot Identity

- **Team:** 2928 (Vikings)
- **Game Year:** 2024 (Crescendo)
- **Network:** 10.29.28.x
- **roboRIO logs path:** `/U/logs`

---

## Hardware Overview

### CAN Bus Topology

| Device | CAN ID | Bus | Type |
|--------|--------|-----|------|
| Pigeon 2 IMU | 0 | `canivore` | CTRE Pigeon2 |
| Front Left Azimuth | 13 | `canivore` | TalonFX (Kraken X60) |
| Front Left Drive | 14 | `canivore` | TalonFX (Kraken X60) |
| Front Left CANcoder | 13 | `canivore` | CANcoder |
| Front Right Azimuth | 19 | `canivore` | TalonFX (Kraken X60) |
| Front Right Drive | 18 | `canivore` | TalonFX (Kraken X60) |
| Front Right CANcoder | 19 | `canivore` | CANcoder |
| Back Left Azimuth | 10 | `canivore` | TalonFX (Kraken X60) |
| Back Left Drive | 11 | `canivore` | TalonFX (Kraken X60) |
| Back Left CANcoder | 10 | `canivore` | CANcoder |
| Back Right Azimuth | 15 | `canivore` | TalonFX (Kraken X60) |
| Back Right Drive | 16 | `canivore` | TalonFX (Kraken X60) |
| Back Right CANcoder | 15 | `canivore` | CANcoder |
| Shooter Pivot | 12 | `canivore` | TalonFX (Kraken X60) |
| Shooter Pivot CANcoder | 12 | `canivore` | CANcoder |
| Shooter Flywheel A | 1 | `canivore` | TalonFX (Kraken X60) |
| Shooter Flywheel B | 4 | `canivore` | TalonFX (Kraken X60) |
| Climber Actuator | 17 | `canivore` | TalonFX (Kraken X60) |
| Feeder/Launcher | 0 | rio (default) | TalonSRX |
| Intake Roller | 3 | rio (default) | TalonSRX |
| Power Distribution Hub | 0 | rio (default) | REV PDH |

### PWM Devices

| Device | PWM Channel |
|--------|-------------|
| Climber Ratchet Servo | 9 |
| Amp Bar Servo A | 0 |
| Amp Bar Servo B | 1 |

### Digital I/O

| Device | DIO Channel | Purpose |
|--------|-------------|---------|
| Release (Setup Jumper) | 0 | Used for diagnostics release mode / coast mode |
| Lockout | 1 | (Defined but not used in current code) |

### Vision Cameras

| Name | Purpose |
|------|---------|
| `limelight-note` | Note detection for ground intake assist |
| `limelight-shooter` | AprilTag tracking for speaker shots (mounted sideways) |
| `limelight-rear` | Rear AprilTag tracking for rear-facing shots |

### LimelightFX LED System

- Connected via USB Serial (port kUSB1, 115200 baud)
- 1x Grid module
- 4x Strip modules (32 LEDs each, daisy-chained in pairs)

---

## Drivetrain

### Physical Configuration

- **Type:** Swerve (4 modules)
- **Module Type:** SDS MK4i L2
- **Wheel Radius:** 2 inches
- **Wheelbase:** 24 inches (29" frame - 2.5" offset × 2)
- **Track Width:** 24 inches (square chassis)
- **Drive Gear Ratio:** (50/14) × (17/27) × (45/15) ≈ 6.746
- **Azimuth Gear Ratio:** 150/7 ≈ 21.43
- **Coupling Ratio:** 3.5714 (azimuth rotation causes drive motor turns)
- **Max Velocity:** 5.0 m/s
- **Max Angular Velocity:** ~6.41 rad/s (computed from max velocity and wheelbase)
- **Robot Mass:** 63.5 kg (used in PathPlanner config)
- **Theoretical Speed at 12V:** 4.73 m/s

### Module Encoder Offsets (Rotations)

| Module | Offset |
|--------|--------|
| Front Left | -0.420654296875 |
| Front Right | 0.299072265625 |
| Back Left | 0.033203125 |
| Back Right | -0.4169921875 |

### Module Positions (from center)

| Module | X (inches) | Y (inches) |
|--------|-----------|-----------|
| Front Left | +12 | +12 |
| Front Right | +12 | -12 |
| Back Left | -12 | +12 |
| Back Right | -12 | -12 |

### Motor Inversions

- **Drive:** Right side (Front Right, Back Right) inverted (Clockwise_Positive)
- **Steer:** Not inverted
- **CANcoder Direction:** CounterClockwise_Positive

### PID & Feedforward

#### Drive Motor (Slot 0)
| Parameter | Value |
|-----------|-------|
| kP | 4.0 |
| kI | 0.0 |
| kD | 0.2 |
| kS | 0.225 |
| kV | 2.62 |
| kA | 0.0 |

#### Azimuth Motor (Slot 0)
| Parameter | Value |
|-----------|-------|
| kP | -60 |
| kI | 0 |
| kD | 0 |
| kS | 0 |
| kV | 0 |
| kA | 0 |

#### Absolute Rotation (Field-Oriented Driving)
| Parameter | Value |
|-----------|-------|
| kP | 2.3 |
| kI | 0 |
| kD | 0.15 |
| Max Velocity (profile) | 1 rot/s |
| Max Accel (profile) | 17 rot/s² |
| Continuous Input Range | [-0.5, 0.5] rotations |

#### Auto Path Following (PathPlanner)
| Controller | kP | kI | kD |
|-----------|-----|-----|-----|
| Translation | 7.5 | 0 | 0.5 |
| Theta | 5.0 | 0 | 0.02 |

### Current Limits

#### Drive Motor
| Parameter | Value |
|-----------|-------|
| Stator Current Limit | 80 A |
| Peak Forward Torque Current | 40 A |
| Peak Reverse Torque Current | -40 A |
| Supply Current Limit | 35 A |
| Duty Cycle Open Loop Ramp | 0.1 s |

#### Azimuth Motor
| Parameter | Value |
|-----------|-------|
| Stator Current Limit | 40 A |
| Peak Forward Torque Current | 40 A |
| Peak Reverse Torque Current | -40 A |
| Supply Current Limit | 35 A |
| Duty Cycle Open Loop Ramp | 0.1 s |

### Neutral Modes
- Drive: **Brake**
- Azimuth: **Brake**

### Theta Compensation
- Factor: 0.2 (rotates translation vector opposite to omega to compensate for drift during rotation)

### Odometry / Pose Estimation
- Uses `SwerveDrivePoseEstimator`
- Vision fusion: Limelight rear pose added when ≥2 AprilTags visible and pose difference < 0.5m (currently commented out)
- Alliance-aware: Pose is mirrored for Red alliance relative to Blue origin

---

## Shooter Subsystem

### Mechanism Description
The shooter is an over-the-bumper pivot mechanism that can:
1. Rotate to ground level to intake notes via a roller
2. Rotate upward to shoot into the speaker (front or rear facing)
3. Rotate to an amp position and use a deployable "amp bar" deflector

### Motors & Sensors

| Component | Hardware | CAN ID | Notes |
|-----------|----------|--------|-------|
| Pivot | TalonFX | 12 (canivore) | Position control via remote CANcoder |
| Pivot Encoder | CANcoder | 12 (canivore) | Absolute position feedback |
| Flywheel A | TalonFX | 1 (canivore) | Coast mode, velocity control |
| Flywheel B | TalonFX | 4 (canivore) | Coast mode, velocity control |
| Feeder | TalonSRX | 0 (rio) | Brake mode, inverted, has limit switch for note detection |
| Intake Roller | TalonSRX | 3 (rio) | Brake mode, inverted |

### Note Detection
- Uses TalonSRX forward limit switch on the feeder motor
- `holdingNote = !sensors.isFwdLimitSwitchClosed()` (active low / normally open)

### Pivot PID (Slot 0)
| Parameter | Value |
|-----------|-------|
| kP | 10 |
| kI | 0 |
| kD | 0.05 |
| kS | 0.025 |
| kG | 0.028 |
| Gravity Type | Arm_Cosine |

### Pivot Current Limit
- Supply Current Limit: 40 A

### Pivot Soft Limits
| Direction | Value |
|-----------|-------|
| Forward (max up) | 0.39 rotations |
| Reverse (min down/intake) | -0.1085 rotations |

### Flywheel PID (Slot 0)
| Parameter | Value |
|-----------|-------|
| kP | 0.05 |
| kI | 0.0 |
| kD | 0.0 |
| kS | 0 |
| kV | 0.015 |
| kA | 0 |

### Key Angles (Pivot Positions)

| Position | Value | Units |
|----------|-------|-------|
| Intake Ground | -0.1085 | rotations |
| Ready Intake (just above ground) | -0.1085 | rotations |
| Ready Drive (stowed with note) | 0 | degrees |
| Ready Shoot Front | 0.122 | rotations |
| Ready Shoot Rear | 125 | degrees |
| Shoot Amp | 110 | degrees |
| Starting Configuration | 90 | degrees |
| Shoot Sub(woofer) | 115 | degrees (tunable) |
| Shoot Ferry | 130 | degrees (tunable) |
| Max angle | 0.39 | rotations |

### Flywheel Configuration (Active: "Green Bane")
| Parameter | Value |
|-----------|-------|
| Speaker Velocity | 40 rot/s |
| Speaker Velocity Threshold | 37 rot/s |
| Amp Power (duty cycle) | 0.60 |

Note: An alternate "Yellow Fairlane" config exists (30/27 rot/s, 0.3 amp power) but is not active.

### Amp Bar Servos
| Action | Servo A | Servo B |
|--------|---------|---------|
| Extend | 1.0 | 0.0 |
| Retract | 0.0 | 1.0 |

### Shoot Conditions (Speaker)
- Flywheel at speed (≥ threshold)
- Pivot velocity < 2 deg/s (`pivotMaxVelocityShoot`)
- Vision alignment confirmed (horizontal offset < tunable threshold, default 1.25°)
- Fire timeout: 0.3 seconds after feeder engages

### Vision-Assisted Shooting
- Limelight mounted sideways: `tx` = pitch offset, `ty` = yaw offset
- Auto-alignment: Uses feedforward on yaw offset to rotate robot toward target (clamped to ±0.125 speed)
- Pitch correction: PID controller (P=0.75) adjusts pivot angle based on limelight `tx`
- Pipeline switching: Pipeline 0 when facing forward, Pipeline 1 otherwise

---

## Climber Subsystem

### Mechanism Description
Single-stage linear climber with a ratchet mechanism for holding position under load.

### Hardware

| Component | Hardware | ID | Notes |
|-----------|----------|-----|-------|
| Actuator | TalonFX | 17 (canivore) | Position control, inverted (CCW+) |
| Ratchet Servo | Servo | PWM 9 | Locks climber under load |
| Home Switch | Remote CANcoder limit | ID 17 | Reverse limit, normally open, auto-zeros position |

### PID
| Config | kP | Purpose |
|--------|-----|---------|
| Fast | 0.25 | Normal operation |
| Slow | 0.01 | (Available but not actively used) |

### Position Constants
| Parameter | Value |
|-----------|-------|
| Max position | 129 (rotations) |
| Disengage distance | 0.5 |
| Initialize raise distance | 2 |

### Ratchet Servo Positions
| State | Angle |
|-------|-------|
| Locked | 84° |
| Free | 93° |

**Note:** Ratchet is currently **disabled** (`ratchetEnabled = false`). The ratchet logic exists but does not engage.

### Initialization Sequence
1. Raise climber at 25% power until it moves `initializeRaiseDistance` (2 rotations) above start
2. Lower at 75% power until home switch triggers
3. Home switch auto-zeros position

### Neutral Mode
- Brake

---

## Operator Interface (Control Scheme)

### Controllers
| Role | Port | Type |
|------|------|------|
| Driver | 0 | Xbox Controller |
| Operator | 1 | Xbox Controller |

### Emergency Stop (Both Controllers)
| Button | Action |
|--------|--------|
| Start (Menu) | **C-Stop** — cancels ALL running commands |

### Driver Controls (Port 0)

| Input | Binding | Action |
|-------|---------|--------|
| Left Stick Y | `driveAxial` | Forward/backward translation |
| Left Stick X | `driveLateral` | Left/right translation |
| Right Stick X | `driveFORX` | Rotation (Swerve mode) / Target direction X (Field Oriented mode) |
| Right Stick Y | `driveFORY` | Target direction Y (Field Oriented mode only) |
| Right Stick Click | `manualRotation` | (Trigger defined but not bound to command) |
| Left Trigger | `shootSpeaker` | **While held:** Vision-assisted speaker shot |
| Left Bumper | `shootAmp` | **Press:** Prepare amp (extend bar, pivot to 10°, 0.3s timeout) → **While held:** Execute amp shot → **Release:** Finish amp (retract bar, 0.6s timeout) |
| Right Trigger | `intake` | **While held:** Ground intake with note-tracking drive assist |
| Right Bumper | `ferry` | **While held:** Fixed-angle shot at tunable ferry angle |
| Y | `resetFOD` | Reset field-oriented heading to current facing |
| X | `lockWheels` | Toggle LED state (lock wheels command commented out) |
| A | (test) | **While held:** TestDrive command |

### Operator Controls (Port 1)

| Input | Binding | Action |
|-------|---------|--------|
| Y | `climberUp` | **While held:** Drive climber to max position (129) |
| X | `climberDown` | **While held:** Drive climber to position 0 |
| D-pad Up | `climberOverrideRaise` | **While held:** Override climber up at 100% duty cycle |
| D-pad Down | `climberOverrideLower` | **While held:** Override climber down at -100% duty cycle |
| Right Stick Click | `initializeClimber` | Trigger climber homing sequence |
| B | `intakeOut` | **While held:** Run intake roller in reverse |
| A | `intakeIn` | **While held:** Run intake roller forward |
| Left Trigger | `fixedShoot` | **While held:** Fixed-angle subwoofer shot (tunable angle) |
| Right Trigger | `overrideShoot` | Override shoot condition (fire even without full alignment) |
| Right Bumper | `foc` | (Trigger defined, not bound to command) |

### Drive Modes (Dashboard Selectable)
1. **Swerve Drive** (default): Right stick X directly controls rotation rate
2. **Field Oriented**: Right stick angle sets target heading; robot rotates to face that direction using profiled PID

### Haptics
- **Intake:** Both rumble motors pulse at 100% while intaking (1s interval, 100% duty cycle)
- **Lock Wheels:** 25% rumble while wheels are locked

---

## Autonomous Routines

### Currently Active (Uncommented)

| Name | Description |
|------|-------------|
| `[comp] Drive` | Ready shooter at rear angle → drive forward 2 m/s for 1 second |
| `[comp] Shoot/Drive` | Ready shooter → shoot speaker (2s timeout) → drive forward 1s |
| `[comp] Shoot` | Ready shooter → shoot speaker (2s timeout) |
| `[comp] Two Note` | Shoot fixed → intake ground (1.5s timeout) → shoot speaker (2s timeout) |
| `[testing] Intake Only` | Ground intake with 2s timeout |
| `[testing] LookForNote` | Rotate ±45° scanning for notes, intake if found |
| `[testing] voltage ramp` | Voltage ramp characterization |

### Auto Init Behavior (hardcoded in Robot.java)
The current `autonomousInit()` ignores the chooser and runs:
1. `LookForNote` (rotate +45°)
2. If note found → `IntakeGround` (4s timeout)
3. If not found → `LookForNote` (rotate -90°)
4. Default command: `LockWheels`

### Commented-Out Path-Following Autos
These used Choreo trajectories + PathPlanner dynamic pathing but are broken due to Choreo 2026 incompatibility:
- **Five Note** (middle start, 5 speaker shots)
- **Source Side Center Note** (source side, 2 notes)
- **Amp Side Center Note** (amp side, 2-3 notes)
- **The Jamp** (amp side variation)
- **The Close Jamp** (front-facing shot variant)
- **The Jource** (source side variation)
- **Source Side Bulldoze** (source side push strategy)

### Auto Driving Constants
- DriveTime speed: 2 m/s forward (robot-relative)
- PathPlanner max acceleration: 2-3 m/s²
- PathPlanner max angular acceleration: 2 rad/s²

---

## Diagnostics & Safety

### Release Mode (Coast Mode)
- Triggered by DIO 0 going low while disabled
- Sets ALL motors (drive, azimuth, shooter pivot, climber) to Coast mode
- Plays audio feedback chirp on engage/disengage

### Pre-Match Checks (Background Thread)
| Check | Alert Level |
|-------|-------------|
| Setup jumper present | ERROR |
| Auto routine not marked `[comp]` | ERROR |
| Shooter not at 90° starting config (>6° off) | ERROR |
| Climber not at home | ERROR |
| Not holding a note | ERROR |
| Battery < 12V | WARNING |

### Audio Feedback
- Boot: 600Hz 500ms → 900Hz 500ms chirp
- Music allowed during disable (TalonFX audio config)
- LimelightFX beep on startup

---

## LED Behaviors (LimelightFX)

| State | Grid | Strips |
|-------|------|--------|
| Disabled | "disabled" image | Solid white |
| Diagnostics issue | — | Solid red |
| Autonomous | Blink | — |
| Teleop | "teleop" image | — |
| Holding note | Solid orange (255,127,0) | Solid orange (255,127,0) |
| Signal drop note | "dropnote" image | Orange/black chevrons (width 2/2) |

---

## Logging & Telemetry

### Framework
- **AdvantageKit** (LoggedRobot, Logger, AutoLog, AutoLogOutput)
- Real mode: WPILog to USB + NT4
- Sim mode: NT4 only
- Replay mode: reads WPILog, writes `_sim` suffix log

### Tunable Parameters (NetworkTables)
| Key | Default | Purpose |
|-----|---------|---------|
| `Tuning/FlywheelSpeed` | 40 rot/s | Speaker flywheel velocity setpoint |
| `Tuning/FlywheelSpeedThreshold` | 37 rot/s | Minimum speed to allow firing |
| `Tuning/AmpAngle` | 110° | Amp shot pivot angle |
| `Tuning/AmpPower` | 0.60 | Amp shot flywheel duty cycle |
| `Tuning/Drivetrain P` | 0 | (Unused tuning slot) |
| `Tuning/ShootSpeakerPivotThreshold` | 1.25° | Min LL offset before pitch correction activates |
| `Tuning/ShootSpeakerExponent` | 1 | Power curve for pitch correction |
| `Tuning/FerryAngle` | 130° | Ferry shot pivot angle |
| `Tuning/SubAngle` | 115° | Subwoofer fixed shot angle |

---

## Command Behavior Summary

### ShootSpeaker
1. Spin up flywheels to tunable velocity
2. Determine robot orientation (facing speaker or away)
3. If facing forward with shooter forward:
   - Use shooter limelight to auto-aim (yaw → robot rotation, pitch → pivot adjustment)
   - Wait for: flywheel at speed + pivot settled + alignment
   - Fire feeder
4. If facing away: use rear limelight, pivot to rear angle
5. Timeout override available
6. On end: pivot to readyDrive (if holding note) or readyIntake (if empty), stop flywheels

### IntakeGround
1. Pivot to intake ground angle
2. Run flywheels in reverse at -25% (to not eject note)
3. Run feeder in reverse
4. Run intake roller forward (only when pivot is within 1.5° of target)
5. If `correction` enabled: use note limelight to steer robot toward detected note
6. Ends when note detected (limit switch)
7. Haptics: full rumble while active

### ShootAmp
1. PrepareAmpShot: extend amp bar, pivot to 10°
2. ShootAmp: pivot to tunable amp angle, run flywheels at tunable amp power
3. Wait for pivot within 20.5° AND driver holds right trigger → fire feeder
4. FinishAmpShot: retract amp bar, pivot to 10°, then settle to readyDrive/readyIntake

### ShootFixed (Subwoofer/Ferry)
1. Spin flywheels to velocity
2. Pivot to specified angle
3. Fire when: angle within 1.25° + flywheel at speed + pivot velocity below threshold
4. Timeout override (for auto)

### Climber Initialize
1. Raise at 25% power for 2 rotations above starting position
2. Lower at 75% until home switch triggers
3. Position auto-zeros on home switch

---

## Vendor Dependencies

| Library | Version/Note |
|---------|------|
| WPILib | 2026 |
| AdvantageKit | Latest |
| CTRE Phoenix 6 | 2026 latest |
| CTRE Phoenix 5 | 2026 latest (for TalonSRX) |
| PathPlannerLib | 2026.1.2 |
| ChoreoLib | 2026 |
| URCL | Latest |

---

## Key Architectural Notes for Migration

1. **No Superstructure exists yet.** Commands directly access `Robot.cont.shooter`, `Robot.cont.drivetrain`, etc. via static references.

2. **Commands own coordination.** ShootSpeaker controls both the shooter AND drivetrain alignment. IntakeGround controls both intake AND drive correction. These need to become unified goals.

3. **Implicit idle behavior.** The shooter `Idle` command (not currently scheduled as default) and the pivot-to-readyDrive/readyIntake logic in `end()` methods define the idle behavior.

4. **Feeder + Intake are part of Shooter.** Despite being separate motors, the feeder and intake roller are controlled through `ShooterIO`. In the new architecture, consider whether these should be their own subsystems or remain part of the shooter.

5. **The climber ratchet logic exists but is disabled.** The mechanism code for ratchet disengage-before-raise is complete but `ratchetEnabled = false`.

6. **Diagnostics Release mode** needs to span all subsystems (sets everything to coast). This maps naturally to a diagnostic override in the new architecture.

7. **The robot drives at -speed** — the `SwerveModule.control()` method negates `speedMetersPerSecond` before applying, and uses manual 180° flip optimization instead of WPILib's `SwerveModuleState.optimize()`.

8. **Vision pose fusion is commented out.** The infrastructure exists but the `addVisionMeasurement` call is disabled.

9. **Autonomous is partially broken.** Choreo trajectories from 2024 are incompatible with 2026 libraries. The auto init is hardcoded to a note-seeking behavior that ignores the chooser.

10. **TalonSRX (Phoenix 5) vs TalonFX (Phoenix 6):** The feeder and intake use legacy TalonSRX motors with percent output control. All other motors are TalonFX with Phoenix 6 closed-loop control.
