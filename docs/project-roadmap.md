# VikingRobot2026 — Architecture Refactor Roadmap

This document tracks the subsystem-by-subsystem refactor to the goal-based architecture defined in `CLAUDE.md`. Each phase must be fully code-complete, build-verified, and hardware-tested before the next phase begins.

---

## Quick Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Drive — goal-based refactor + hardware test | 🔄 Code complete, awaiting hardware |
| 2 | Superstructure skeleton + Drive wired | 🔄 Code complete, awaiting hardware |
| 3 | Shooter — full conversion (state machine + Superstructure + OI) | ⬜ Pending |
| 4 | Climber — full conversion (state machine + Superstructure + OI) | ⬜ Pending |
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
- [ ] Deploy to robot, no CAN faults on boot
- [ ] All 4 swerve modules respond in teleop (correct direction, no oscillation)
- [ ] Field-oriented drive works correctly (push stick forward → robot moves away from driver regardless of heading)
- [ ] `resetAngle()` (Y button) zeros field-oriented heading
- [ ] Wheel lock (X button): `Drive/SystemState` logs `LOCK`, robot holds on an incline, controller rumbles; reverts to `TELEOP` on release
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
- [ ] `./gradlew build` passes clean (pending WPILib VS Code build)

### Hardware Test Checklist
- [ ] Teleop drive behavior identical to Phase 1 (joystick, lock, mode chooser)
- [ ] AdvantageScope: `Superstructure/Goal` logs `DriveGoal` each cycle
- [ ] Wheel lock still works via intent path

---

## Phase 3 — Shooter Full Conversion

**Status:** ⬜ Pending Phase 2 hardware sign-off

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
- [ ] Create `ShooterGoal.java` enum
- [ ] Add `WantedState`/`SystemState` enums and `applyGoal(ShooterGoal)` to `Shooter.java`
- [ ] Implement `handleStateTransition()` (sensor checks → `SystemState`)
- [ ] Implement `applyState()` (calls `io.*` methods based on `SystemState`)
- [ ] Update `periodic()` to drive the state machine
- [ ] Add `ShooterGoal shooter` field to `RobotGoal` (safe default: `ShooterGoal.HOME`)
- [ ] Add shooter state to `SuperstructureContext`
- [ ] Update `GoalResolver` with shooter intents
- [ ] Update OI to push shooter intents; remove/update redundant command files
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Pivot moves to home on enable; correct angle for each goal
- [ ] Flywheels spin up to correct velocity; feeder fires only when at speed
- [ ] Amp bar extends/retracts correctly
- [ ] AdvantageScope: `Shooter/SystemState`, `Shooter/WantedState`, `Shooter/Angle`, `Shooter/FlywheelSpeed` log correctly
- [ ] `IntakeGround` still updates `drivetrain.joystickSpeeds` correctly (vision correction path)

---

## Phase 4 — Climber Full Conversion

**Status:** ⬜ Pending Phase 3 hardware sign-off

### Goal
Convert `Climber.java` to the WantedState/SystemState pattern, wire into the Superstructure, and implement the key interlock: climber cannot deploy if the shooter is not at a safe angle. The interlock lives in `Climber.applyGoal()` as a guard clause checking `SuperstructureContext`.

### Relevant Files
- `src/main/java/frc/robot/subsystems/Climber.java` — add state machine + interlock
- `src/main/java/frc/robot/subsystems/ClimberIO.java` — interface (likely unchanged)
- `src/main/java/frc/robot/subsystems/ClimberIOReal.java` — hardware impl (likely unchanged)
- Create `src/main/java/frc/robot/subsystems/climber/ClimberGoal.java`
- `src/main/java/frc/robot/superstructure/RobotGoal.java` — add `ClimberGoal climber` field
- `src/main/java/frc/robot/superstructure/SuperstructureContext.java` — add climber state snapshot
- `src/main/java/frc/robot/superstructure/GoalResolver.java` — add climber intents
- `src/main/java/frc/robot/oi/OperatorOI.java` — climber triggers use intents

### Hardware: What's on the Climber
- TalonFX: actuator motor (with remote CANcoder for position feedback)
- Servo: ratchet lock

### Proposed ClimberGoal States
| Goal | Meaning |
|------|---------|
| `IDLE` | Hold current position, ratchet engaged. Safe default. |
| `INITIALIZE` | Run to home (limit switch) and zero encoder |
| `DEPLOY` | Extend to max height |
| `RETRACT` | Pull down to climb position |
| `LOCKED` | Ratchet engaged, motor off |

### Interlock
```java
// In Climber.applyGoal():
if (desired == ClimberGoal.DEPLOY && !ctx.shooterState().isAtSafeAngle()) {
    wantedState = WantedState.IDLE; // wait for shooter to clear
    return;
}
```

### Checklist
- [ ] Create `ClimberGoal.java` enum
- [ ] Add `WantedState`/`SystemState` enums and `applyGoal(ClimberGoal)` to `Climber.java`
- [ ] Implement `handleStateTransition()` (position/limit checks → `SystemState`)
- [ ] Implement `applyState()` (calls `io.*` methods)
- [ ] Add deploy interlock using `SuperstructureContext.shooterState().isAtSafeAngle()`
- [ ] Add `ClimberGoal climber` field to `RobotGoal` (safe default: `ClimberGoal.IDLE`)
- [ ] Add climber state to `SuperstructureContext`
- [ ] Update `GoalResolver` with climber intents
- [ ] Update `OperatorOI` to push climber intents
- [ ] `./gradlew build` passes clean

### Hardware Test Checklist
- [ ] Climber initializes to home correctly
- [ ] Deploy extends to full height; blocked when shooter is not at safe angle, unblocks when it clears
- [ ] Retract pulls down smoothly; ratchet engages under load
- [ ] AdvantageScope: `Climber/SystemState`, `Climber/Position` log correctly

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
- **FinishAmpShot / PrepareAmpShot**: These commands require `drivetrain` as a requirement even though they only pass joystick speeds through. This is a leaky abstraction to address during Phase 3 OI migration.
