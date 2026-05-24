# VikingRobot2026 — Architecture Refactor Roadmap

This document tracks the subsystem-by-subsystem refactor to the goal-based architecture defined in `CLAUDE.md`. Each phase must be fully code-complete, build-verified, and hardware-tested before the next phase begins.

---

## Quick Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Drive — hardware test | 🔄 In Progress |
| 2 | Shooter — subsystem conversion | ⬜ Pending |
| 3 | Climber — subsystem conversion | ⬜ Pending |
| 4 | Superstructure + RobotGoal | ⬜ Pending |
| 5 | OI migration | ⬜ Pending |
| 6 | Auto routines | ⬜ Pending |
| 7 | Architecture docs | ⬜ Pending |

---

## Phase 1 — Drive Hardware Test

**Status:** 🔄 Code complete, awaiting robot time

### Goal
Validate that the CTRE `SwerveDrivetrain` migration works correctly on real hardware. The code conversion is done; this phase is purely about physical verification before more code is written on top of it.

### What Was Changed
- Replaced custom `Drivetrain.java` / `SwerveModule.java` / `ModuleIO*` / `GyroIO*` stack with CTRE `SwerveDrivetrain` base class
- New files: `subsystems/drive/DriveSubsystem.java`, `DriveGoal.java`, `TunerConstants.java`
- `applyGoal(DriveGoal)` stub added — ready for Superstructure wiring later
- Rear Limelight pose fusion re-enabled (MegaTag2 best practices: reject if omega > 360 deg/s, require ≥2 tags)
- PathPlanner `AutoBuilder` configured in constructor

### Hardware Test Checklist
- [ ] Deploy to robot, no CAN faults on boot
- [ ] All 4 swerve modules respond in teleop (correct direction, no oscillation)
- [ ] Field-oriented drive works correctly (push stick forward → robot moves away from driver regardless of heading)
- [ ] `resetAngle()` (Y button) zeros field-oriented heading
- [ ] Wheel lock (X button) holds robot in place on an incline
- [ ] AdvantageScope: `Drive/Pose`, `Drive/ModuleStates`, `Drive/ModuleTargets` log correctly
- [ ] PathPlanner auto: robot follows a simple path from correct start pose
- [ ] Rear Limelight: `Drive/VisionPoseAccepted` logs true when tags are visible and robot is not spinning

---

## Phase 2 — Shooter Subsystem Conversion

**Status:** ⬜ Pending Phase 1 hardware sign-off

### Goal
Add `WantedState`/`SystemState` enums and `applyGoal(ShooterGoal)` to `Shooter.java`, establishing the internal state machine. Commands still call subsystem methods directly at this stage — the Superstructure wiring comes in Phase 4.

### Relevant Files
- `src/main/java/frc/robot/subsystems/Shooter.java` — add state machine
- `src/main/java/frc/robot/subsystems/ShooterIO.java` — interface (likely unchanged)
- `src/main/java/frc/robot/subsystems/ShooterIOReal.java` — hardware impl (likely unchanged)
- Create `src/main/java/frc/robot/subsystems/shooter/ShooterGoal.java`

### Hardware: What's on the Shooter
- TalonFX: pivot motor (with CANcoder for absolute position)
- TalonFX × 2: flywheels A and B
- TalonSRX: feeder and intake roller
- Servo: amp bar (extend/retract)

### Proposed States
| WantedState | Meaning |
|-------------|---------|
| `HOME` | Pivot down, flywheels off, feeder stopped |
| `INTAKE` | Pivot at intake angle, intake roller running |
| `READY_DRIVE` | Pivot at drive-safe angle, flywheels off |
| `SPIN_UP` | Pivot targeting, flywheels ramping to velocity |
| `SHOOT` | Flywheels at speed, feeder fires |
| `AMP` | Amp bar extended, pivot at amp angle, flywheels at amp power |
| `FERRY` | Pivot at ferry angle, flywheels at ferry power |

### Conversion Checklist
- [ ] Create `ShooterGoal.java` enum
- [ ] Add `WantedState` / `SystemState` enums inside `Shooter.java`
- [ ] Implement `applyGoal(ShooterGoal)` (sets `wantedState`; no interlock logic yet)
- [ ] Implement `handleStateTransition()` (sensor checks → `SystemState`)
- [ ] Implement `applyState()` (calls `io.*` methods based on `SystemState`)
- [ ] Update `periodic()` to call `handleStateTransition()` → `applyState()`
- [ ] Existing commands (`ReadyShooter`, `ShootSpeaker`, `IntakeGround`, etc.) continue to call `io.*` directly — no command changes in this phase
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Pivot moves to home position on enable
- [ ] Pivot reaches and holds intake angle during IntakeGround command
- [ ] Flywheels spin up to correct velocity for ShootSpeaker
- [ ] Feeder fires when commanded
- [ ] Amp bar extends/retracts on PrepareAmpShot/FinishAmpShot
- [ ] AdvantageScope: `Shooter/SystemState`, `Shooter/WantedState`, `Shooter/Angle`, `Shooter/FlywheelSpeed` all log correctly

---

## Phase 3 — Climber Subsystem Conversion

**Status:** ⬜ Pending Phase 2 hardware sign-off

### Goal
Add `WantedState`/`SystemState` enums and `applyGoal(ClimberGoal)` to `Climber.java`. The key physical constraint — climber cannot deploy while the intake/shooter is in a conflicting position — is noted here but will be enforced as an interlock in Phase 4.

### Relevant Files
- `src/main/java/frc/robot/subsystems/Climber.java` — add state machine
- `src/main/java/frc/robot/subsystems/ClimberIO.java` — interface (likely unchanged)
- `src/main/java/frc/robot/subsystems/ClimberIOReal.java` — hardware impl (likely unchanged)
- Create `src/main/java/frc/robot/subsystems/climber/ClimberGoal.java`

### Hardware: What's on the Climber
- TalonFX: actuator motor (with remote CANcoder for position feedback)
- Servo: ratchet lock

### Proposed States
| WantedState | Meaning |
|-------------|---------|
| `IDLE` | Hold current position, ratchet engaged |
| `INITIALIZE` | Run to home (limit switch) and zero encoder |
| `DEPLOY` | Extend to max height |
| `RETRACT` | Pull down to climb position |
| `LOCKED` | Ratchet engaged, motor off |

### Conversion Checklist
- [ ] Create `ClimberGoal.java` enum
- [ ] Add `WantedState` / `SystemState` enums inside `Climber.java`
- [ ] Implement `applyGoal(ClimberGoal)` (sets `wantedState`)
- [ ] Implement `handleStateTransition()` with home/limit detection
- [ ] Implement `applyState()` (calls `io.*` methods)
- [ ] Update `periodic()`
- [ ] Existing operator controls continue working via `OperatorOI` commands — no OI changes in this phase
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Climber initializes to home position correctly
- [ ] Deploy extends to full height
- [ ] Retract pulls down smoothly
- [ ] Ratchet engages at end of retract (robot holds position under load)
- [ ] AdvantageScope: `Climber/SystemState`, `Climber/Position`, `Climber/Home` log correctly

---

## Phase 4 — Superstructure + RobotGoal

**Status:** ⬜ Pending Phases 2 & 3 hardware sign-off

### Goal
Build the central coordination layer. `Superstructure` produces a `RobotGoal` each cycle and calls `applyGoal()` on every subsystem. Interlocks move from commands into `applyGoal()` guard clauses inside each subsystem.

### New Files
- `src/main/java/frc/robot/superstructure/Superstructure.java`
- `src/main/java/frc/robot/superstructure/RobotGoal.java` (builder pattern)
- `src/main/java/frc/robot/superstructure/SuperstructureContext.java`
- `src/main/java/frc/robot/superstructure/GoalResolver.java`

### Key Interlocks to Implement
| Subsystem | Condition | Guard |
|-----------|-----------|-------|
| Climber | Cannot deploy if shooter not at safe angle | `ClimberSubsystem.applyGoal()` checks `ctx.shooterState()` |
| Shooter | Cannot fire if flywheels not at speed | internal `handleStateTransition()` — already enforced by state machine |

### Checklist
- [ ] Design `RobotGoal` record with one field per subsystem goal, builder with safe defaults
- [ ] Create `SuperstructureContext` (immutable snapshot of all subsystem states)
- [ ] Create `GoalResolver` (reads OI intents → produces `RobotGoal`)
- [ ] Create `Superstructure.periodic()` loop: resolve goal → build context → call `applyGoal()` on all subsystems
- [ ] Add `Superstructure.setGoalCommand(RobotGoal)` for use by auto routines
- [ ] Add interlock guard clauses to `ClimberSubsystem.applyGoal()` and `ShooterSubsystem.applyGoal()`
- [ ] Register `Superstructure` in `RobotContainer`
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Teleop: all subsystems behave identically to pre-Superstructure behavior
- [ ] Interlock: climber deploy is blocked when shooter is at a conflicting angle; unblocks when shooter clears
- [ ] AdvantageScope: `Superstructure/Goal`, `Superstructure/Context` log each cycle

---

## Phase 5 — OI Migration

**Status:** ⬜ Pending Phase 4 hardware sign-off

### Goal
Replace direct subsystem/command calls in `DriverOI` and `OperatorOI` with Superstructure intents. Commands like `ShootSpeaker` and `IntakeGround` become goal setters rather than direct IO callers.

### Files to Update
- `src/main/java/frc/robot/oi/DriverOI.java`
- `src/main/java/frc/robot/oi/OperatorOI.java`
- Shooter commands: `ShootSpeaker.java`, `IntakeGround.java`, `ShootAmp.java`, etc.

### Checklist
- [ ] OI triggers call `superstructure.pushIntent(Intent.SHOOT_SPEAKER)` etc. instead of scheduling commands
- [ ] `GoalResolver` maps intents → `RobotGoal` instances
- [ ] Remove `drivetrain` / `shooter` / `climber` direct references from OI classes
- [ ] Existing command files updated or deleted as goals replace them
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] All driver controls produce correct robot behavior
- [ ] All operator controls produce correct robot behavior
- [ ] No command requirement conflicts (subsystem scheduling errors)

---

## Phase 6 — Auto Routines

**Status:** ⬜ Pending Phase 5 hardware sign-off

### Goal
Update `Autonomous.java` routines to use `Superstructure.setGoalCommand()` instead of commanding subsystems directly. Re-enable and expand the commented-out routes.

### Files to Update
- `src/main/java/frc/robot/Autonomous.java`

### Current Active Routes
- `[comp] Drive` — drive forward 1s
- `[comp] Shoot/Drive` — ready shooter, shoot, drive
- `[comp] Shoot` — ready shooter, shoot
- `[comp] Two Note` — shoot fixed, intake, shoot speaker
- `[testing] Intake Only` — intake 2s
- `[testing] LookForNote` — find note, intake

### Checklist
- [ ] Replace command-based sequences with `superstructure.setGoalCommand(RobotGoal.*)` calls
- [ ] Re-enable commented-out auto paths (multi-note, position-based)
- [ ] Verify PathPlanner/Choreo integration still works with goal-based drive
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Each active auto route runs correctly in a practice field setup
- [ ] Robot starts in correct pose for each route
- [ ] Paths follow expected trajectories (verify in AdvantageScope Field2d)

---

## Phase 7 — Architecture Docs

**Status:** ⬜ Ongoing (write each doc as the corresponding code is completed)

### Goal
Produce student-readable reference docs in `docs/architecture/`. Each doc should take under 5 minutes to read and include a Mermaid diagram where helpful.

| Doc | Write After | Status |
|-----|-------------|--------|
| `docs/architecture/superstructure.md` | Phase 4 | ⬜ |
| `docs/architecture/subsystem-template.md` | Phase 2 | ⬜ |
| `docs/architecture/interlocks.md` | Phase 4 | ⬜ |
| `docs/architecture/auto-routines.md` | Phase 6 | ⬜ |
| `docs/architecture/vision.md` | Phase 1 | ⬜ |

---

## Known Issues / Notes

- **LimelightHelpers version**: Current version (v1.2.1) does not expose the full MegaTag2 API (`getBotPoseEstimate_wpiBlue_MegaTag2`). Vision fusion currently uses `Limelight.getPose2d()` as a fallback. Upgrading LimelightHelpers will unlock proper MegaTag2 timestamp and tag-count fields.
- **VoltageRampCommand**: `runCharacterization()` was removed in the CTRE migration. If SysId characterization is needed, use CTRE's built-in SysId routines via `SwerveDrivetrain.sysIdQuasistatic()` / `sysIdDynamic()`.
- **ShooterSpeaker interlock**: The old code checked `drivetrain.est.getEstimatedPosition()` for facing direction. This now uses `drivetrain.getPose()` — verify field-oriented facing logic is still correct during Phase 2 hardware testing.
- **FinishAmpShot / PrepareAmpShot**: These commands require `drivetrain` as a requirement even though they only pass joystick speeds through. This is a leaky abstraction to address during Phase 5 OI migration.
