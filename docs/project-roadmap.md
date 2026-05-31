# VikingRobot2026 — Architecture Refactor Roadmap

This document tracks the subsystem-by-subsystem refactor to the goal-based architecture defined in `CLAUDE.md`. Each phase must be fully code-complete, build-verified, and hardware-tested before the next phase begins.

---

## Quick Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Drive — goal-based refactor + hardware test | 🔄 Partial hardware testing done |
| 2 | Superstructure skeleton + Drive wired | 🔄 Partial hardware testing done |
| 3 | Shooter — full conversion (state machine + Superstructure + OI) | 🔄 Partial hardware testing done |
| 4 | Climber — full conversion (state machine + Superstructure + OI) | 🔄 Code complete, awaiting hardware testing |
| 5 | Auto routines | ⬜ Pending |
| 6 | Architecture docs | ⬜ Ongoing |

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
- **Goal-based drive complete:** `JoystickDrive` and `LockWheels` commands deleted; all drive behavior (joystick computation, absolute-rotation PID, X-lock) moved into `DriveSubsystem.applyGoal(DriveGoal)` / `periodic()`. Default command is now `Commands.run(() -> drivetrain.applyGoal(DriveGoal.TELEOP))`. During autonomous, TELEOP goal automatically becomes LOCK so the robot X-locks when no auto command owns the drivetrain.

### Hardware Test Checklist
- [x] Deploy to robot, no CAN faults on boot
- [x] All 4 swerve modules respond in teleop (correct direction, no oscillation)
- [x] Field-oriented drive works correctly (push stick forward → robot moves away from driver regardless of heading)
- [x] `resetAngle()` (Y button) zeros field-oriented heading
- [x] Wheel lock (X button): `Drive/SystemState` logs `LOCK`, robot holds on an incline, controller rumbles; reverts to `TELEOP` on release
- [ ] AdvantageScope: `Drive/Pose`, `Drive/ModuleStates`, `Drive/ModuleTargets`, `Drive/SystemState` log correctly
- [ ] Drive mode chooser: "Swerve Drive" and "Field Oriented" both work from dashboard
- [ ] PathPlanner auto: robot follows a simple path from correct start pose
- [ ] Rear Limelight: `Drive/VisionPoseAccepted` logs true when tags are visible and robot is not spinning

---

## Phase 2 — Superstructure Skeleton + Drive Wired

**Status:** 🔄 Code complete, awaiting robot time

### Goal
Stand up the minimum viable Superstructure using Drive — which is already fully converted — as the first real consumer. This validates the complete data flow (`RobotGoal` → `SuperstructureContext` → `applyGoal()`) on hardware before any other subsystem is touched.

### Why before Shooter/Climber
The alternative — converting Shooter and Climber first, then adding the Superstructure — creates a double-migration: `applyGoal()` exists on subsystems but is dead code until Phase 4, and every subsystem gets touched twice. Building the shell now means Shooter and Climber each go from "commands call io.* directly" to "fully integrated" in one pass (Phases 3 and 4).

### What Was Changed
- `DriveSubsystem.applyGoal(DriveGoal)` → `applyGoal(SuperstructureContext ctx)`, extracting drive goal via `ctx.goal().drive()`. Signature now matches the full pattern required by Shooter and Climber.
- New package `frc.robot.superstructure` with four files:
  - `RobotGoal.java` — record with builder, safe default `DriveGoal.TELEOP`, factory methods `freeDrive()` / `lockWheels()` / `autonomous()`, and `modify()` for derivation
  - `SuperstructureContext.java` — immutable record wrapping `RobotGoal`; subsystem state snapshots added in Phase 3/4
  - `GoalResolver.java` — stores per-subsystem intents (IntentStore role) and resolves them to a `RobotGoal`; forces `AUTONOMOUS` goal when `DriverStation.isAutonomous()` is true
  - `Superstructure.java` — `SubsystemBase`; `periodic()` drives `resolve() → context → applyGoal()`; exposes `setDriveIntentCommand()` for OI and `setGoalCommand()` for auto routines
- `RobotContainer`: removed drivetrain default command; added `public final Superstructure superstructure`; `Superstructure.periodic()` is the sole authority each cycle
- `DriverOI.lockWheels`: replaced `Commands.run(() -> drivetrain.applyGoal(LOCK), drivetrain)` with `superstructure.setDriveIntentCommand(DriveGoal.LOCK)` — intent-based, no drivetrain requirement
- `Robot.java` required no changes — `Superstructure` is a `SubsystemBase` and registers automatically

### Naming resolution
`GoalResolver` serves as both the `IntentStore` (stores OI intents) and the resolver (maps intents + DriverStation state → `RobotGoal`). The two roles can be split into separate classes if complexity warrants it in a later phase.

### Checklist
- [x] Update `DriveSubsystem.applyGoal(DriveGoal)` → `applyGoal(SuperstructureContext ctx)`
- [x] Create `RobotGoal` record with builder and safe defaults
- [x] Create `SuperstructureContext`
- [x] Create `GoalResolver` with `setDriveIntent()` / `resolve()`
- [x] Create `Superstructure.periodic()` and intent/goal command methods
- [x] Register in `RobotContainer`; remove drivetrain default command
- [x] Update `DriverOI.lockWheels` to use intent path
- [x] `./gradlew build` passes clean (pending WPILib VS Code build)

### Hardware Test Checklist
- [x] Teleop drive behavior identical to Phase 1 (joystick, lock, mode chooser)
- [ ] AdvantageScope: `Superstructure/Goal` logs `DriveGoal` each cycle
- [x] Wheel lock still works via intent path

---

## Phase 3 — Shooter Full Conversion

**Status:** 🔄 Code complete, awaiting hardware sign-off

### Goal
Convert `Shooter.java` to the WantedState/SystemState pattern, wire it into the Superstructure, and update OI — all in one pass. When this phase is done, no shooter command calls `io.*` directly; everything routes through `applyGoal(ShooterGoal)`.

**Intake is part of this subsystem.** The intake roller and pivot are mechanically on the shooter and never operate independently. There is no separate Intake subsystem or IntakeGoal — all intake behavior is expressed through `ShooterGoal.INTAKE`.

### Relevant Files
- `src/main/java/frc/robot/subsystems/Shooter.java` — add state machine
- `src/main/java/frc/robot/subsystems/ShooterIO.java` — interface (likely unchanged)
- `src/main/java/frc/robot/subsystems/ShooterIOReal.java` — hardware impl (likely unchanged)
- Create `src/main/java/frc/robot/subsystems/shooter/ShooterGoal.java`
- `src/main/java/frc/robot/superstructure/RobotGoal.java` — add `ShooterGoal shooter` field
- `src/main/java/frc/robot/superstructure/SuperstructureContext.java` — add shooter state snapshot
- `src/main/java/frc/robot/superstructure/GoalResolver.java` — add shooter intents
- `src/main/java/frc/robot/oi/DriverOI.java` — shooter triggers use intents
- Shooter commands (`ShootSpeaker`, `IntakeGround`, `ShootAmp`, etc.) — update or delete

### Hardware: Shooter + Intake (combined subsystem)
- TalonFX: pivot motor (with CANcoder for absolute position)
- TalonFX × 2: flywheels A and B
- TalonSRX: feeder roller
- TalonSRX: intake roller (on same subsystem — never independent)
- Servo: amp bar (extend/retract)

### Proposed ShooterGoal States
| Goal | Meaning |
|------|---------|
| `HOME` | Pivot down, flywheels off, feeder stopped. Safe default. |
| `INTAKE` | Pivot at intake angle, intake roller running |
| `READY_DRIVE` | Pivot at drive-safe angle, flywheels off |
| `SPIN_UP` | Pivot targeting, flywheels ramping to velocity |
| `SHOOT` | Flywheels at speed, feeder fires |
| `AMP` | Amp bar extended, pivot at amp angle, flywheels at amp power |
| `FERRY` | Pivot at ferry angle, flywheels at ferry power |

### Checklist
- [x] Create `ShooterGoal.java` enum (`HOME`, `INTAKE`, `SHOOT_SPEAKER`, `SHOOT_FIXED`, `AMP`, `FERRY`)
- [x] Add `WantedState`/`SystemState` enums and `applyGoal(ShooterGoal)` to `Shooter.java`
- [x] Implement `handleStateTransition()` (sensor checks → `SystemState`; `firedTime` pattern for shot completion)
- [x] Implement `applyState()` (calls `io.*` methods based on `SystemState`)
- [x] Update `periodic()` to drive the state machine
- [x] Add `DriveGoal.AIM_SPEAKER` to `DriveSubsystem` for limelight rotational correction during speaker shots
- [x] Add `ShooterGoal shooter` field to `RobotGoal` (safe default: `ShooterGoal.HOME`)
- [x] Add `ShooterState` snapshot to `SuperstructureContext`
- [x] Update `GoalResolver` with shooter intents (`setShooterIntent`, `shooterIntent` storage)
- [x] Update `Superstructure`: add `Shooter` parameter, `setShooterIntentCommand()`, wire `shooter.applyGoal(ctx)` in `periodic()`
- [x] Update `DriverOI` to use intent commands; replace `ShootSpeaker`/`IntakeGround`/`ShootAmp`/`FinishAmpShot`/`PrepareAmpShot`/`ShootFixed` bindings
- [x] Update `OperatorOI` `fixedShoot` to use `setShooterIntentCommand(SHOOT_FIXED)`
- [x] Delete `PrepareAmpShot`, `ShootAmp`, `FinishAmpShot`, `Idle`, `ShootFixedDiag` command files
- [x] `./gradlew build` passes clean

### Notes
- `ShootSpeaker`, `IntakeGround`, `ShootFixed`, `ReadyShooter` kept — still referenced in `Autonomous.java` active code (Phase 5 cleanup)
- Limelight access in `Shooter.applyShootSpeaker()` goes via `Robot.cont.drivetrain.limelightShooter` (known pragmatic shortcut; proper fix is `DriveState` snapshot in `SuperstructureContext` — Phase 4/5)
- `Logger.processInputs("Shooter", inputs)` commented out pending `./gradlew build` annotation processing generating `ShooterIOInputsAutoLogged`

### Hardware Test Checklist
- [x] Pivot moves to correct angle for intake and amp goals; home on enable
- [ ] Flywheels spin up to correct velocity; feeder fires only when at speed (not yet tested — no shoot goal tuned)
- [x] Amp bar extends/retracts correctly; `ShooterGoal.AMP` functional
- [ ] AdvantageScope: `Shooter/SystemState`, `Shooter/WantedState`, `Shooter/Angle`, `Shooter/FlywheelSpeed` log correctly
- [x] Drive-to-note vision correction functional during `ShooterGoal.INTAKE` (via `DriveGoal.TRACK_NOTE`)
- [ ] Ferry shot not yet tested
- [ ] Shoot speaker not yet tested (pending shoot goal tuning)

---

## Phase 4 — Climber Full Conversion

**Status:** 🔄 Code complete, awaiting hardware testing

### Goal
Convert `Climber.java` to the WantedState/SystemState pattern, wire into the Superstructure, and implement the key interlock: climber cannot deploy if the shooter is not at a safe angle. The interlock lives in `Climber.applyGoal()` as a guard clause checking `SuperstructureContext`.

### What Was Changed
- `ClimberIOReal.java` — stripped non-functional ratchet servo and state machine entirely; `set()` now drives motor directly
- `ClimberIO.java` — removed `periodic()` default method (only existed for ratchet)
- `Constants.Climber` — removed dead ratchet constants (`ratchetEnabled`, `ratchetLocked`, `ratchetFree`, `disengageDistance`)
- Created `src/main/java/frc/robot/subsystems/climber/ClimberGoal.java` — `IDLE`, `INITIALIZE`, `DEPLOY`, `RETRACT`, `LOCKED`
- `Climber.java` — rewritten with full `WantedState`/`SystemState` machine; `applyGoal()` includes shooter-angle interlock; `getState()` returns `ClimberState` snapshot; `Initialize.java` logic folded into `INITIALIZING_RAISE`/`INITIALIZING_HOME` states
- `RobotGoal.java` — added `ClimberGoal climber` field, builder support, `climb()` factory
- `SuperstructureContext.java` — added `ClimberState climberState` field
- `GoalResolver.java` — added `climberIntent` field, `setClimberIntent()`, wired into `resolve()` and `setGoal()`
- `Superstructure.java` — added `Climber` parameter, wires `climber.applyGoal(ctx)` each cycle
- `RobotContainer.java` — `climber` instantiated before `superstructure` (required for constructor)
- `OperatorOI.java` — `climberUp`/`climberDown`/`initializeClimber` converted to intent path; override POV controls remain direct
- Deleted `commands/climber/Initialize.java`

### Hardware: What's on the Climber
- TalonFX: actuator motor (with remote CANcoder for position feedback)

### ClimberGoal States
| Goal | Meaning |
|------|---------|
| `IDLE` | Hold current position. Safe default. |
| `INITIALIZE` | Raise by `initializeRaiseDistance`, then descend until home switch closes and encoder zeros. |
| `DEPLOY` | Extend to `Constants.Climber.max`. |
| `RETRACT` | Pull down to position 0. |
| `LOCKED` | Motor output zero. |

### Interlock
```java
// In Climber.applyGoal():
if (desired == ClimberGoal.DEPLOY && !ctx.shooterState().atSafeAngle()) {
    wantedState = WantedState.IDLE; // wait for shooter to clear
    return;
}
```

### Checklist
- [x] Create `ClimberGoal.java` enum
- [x] Add `WantedState`/`SystemState` enums and `applyGoal(ClimberGoal)` to `Climber.java`
- [x] Implement `handleStateTransition()` (position/limit checks → `SystemState`)
- [x] Implement `applyState()` (calls `io.*` methods)
- [x] Add deploy interlock using `SuperstructureContext.shooterState().atSafeAngle()`
- [x] Add `ClimberGoal climber` field to `RobotGoal` (safe default: `ClimberGoal.IDLE`)
- [x] Add climber state to `SuperstructureContext`
- [x] Update `GoalResolver` with climber intents
- [x] Update `OperatorOI` to push climber intents
- [x] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Climber initializes to home correctly
- [ ] Deploy extends to full height; blocked when shooter is not at safe angle, unblocks when it clears
- [ ] Retract pulls down smoothly
- [ ] AdvantageScope: `Climber/SystemState`, `Climber/WantedState`, `Climber/Position` log correctly

---

## Phase 5 — Auto Routines

**Status:** ⬜ Pending Phase 4 hardware sign-off

### Goal
Update `Autonomous.java` routines to use `Superstructure.setGoalCommand(RobotGoal.*)` instead of commanding subsystems directly. Re-enable and expand the commented-out routes.

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

## Phase 6 — Architecture Docs

**Status:** ⬜ Ongoing (write each doc as the corresponding code is completed)

### Goal
Produce student-readable reference docs in `docs/architecture/`. Each doc should take under 5 minutes to read and include a Mermaid diagram where helpful.

| Doc | Write After | Status |
|-----|-------------|--------|
| `docs/architecture/superstructure.md` | Phase 2 | ⬜ |
| `docs/architecture/subsystem-template.md` | Phase 3 | ⬜ |
| `docs/architecture/interlocks.md` | Phase 4 | ⬜ |
| `docs/architecture/auto-routines.md` | Phase 5 | ⬜ |
| `docs/architecture/vision.md` | Phase 1 | ⬜ |

---

## Known Issues / Notes

- ~~**`applyGoal()` signature mismatch**~~ — **Resolved in Phase 2.** `DriveSubsystem` now uses `applyGoal(SuperstructureContext ctx)`.
- ~~**`GoalResolver` vs `IntentStore`**~~ — **Resolved in Phase 2.** `GoalResolver` serves both roles; see Phase 2 naming resolution note above.
- **Intake is part of Shooter**: There is no separate Intake subsystem or IntakeGoal. All intake behaviour is part of `Shooter.java` and `ShooterGoal`. `CLAUDE.md` has been updated to reflect this.
- **LimelightHelpers version**: Current version (v1.2.1) does not expose the full MegaTag2 API (`getBotPoseEstimate_wpiBlue_MegaTag2`). Vision fusion currently uses `Limelight.getPose2d()` as a fallback. Upgrading LimelightHelpers will unlock proper MegaTag2 timestamp and tag-count fields.
- **VoltageRampCommand**: `runCharacterization()` was removed in the CTRE migration. If SysId characterization is needed, use CTRE's built-in SysId routines via `SwerveDrivetrain.sysIdQuasistatic()` / `sysIdDynamic()`.
- **ShooterSpeaker interlock**: The old code checked `drivetrain.est.getEstimatedPosition()` for facing direction. This now uses `drivetrain.getPose()` — verify field-oriented facing logic is still correct during Phase 1 hardware testing.
- ~~**FinishAmpShot / PrepareAmpShot**~~ — **Resolved in Phase 3.** These commands and `ShootAmp`, `Idle`, `ShootFixedDiag` were deleted. Amp behavior is now handled entirely within `Shooter.applyAmp()` / `applyHome()`.
- **Climber ratchet removed**: The ratchet servo was non-functional and has been deleted from `ClimberIOReal`. `ClimberGoal.LOCKED` now means motor output zero only (no servo). If a ratchet is added to the 2026 robot, a new IO implementation and servo logic will be needed.
- **`ClimberIOInputsAutoLogged`**: `Logger.processInputs("Climber", inputs)` is commented out in `Climber.java` pending `./gradlew build` generating the AutoLogged class. Uncomment and update `inputs` field type after the first full build.
