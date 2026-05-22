# API Reference Guide

When implementing code that uses WPILib or CTRE Phoenix 6, **always consult the relevant documentation** before writing or suggesting code. Do not rely on training data alone — APIs change between versions.

---

## WPILib

| Resource | URL |
|---|---|
| Java API Index | https://github.wpilib.org/allwpilib/docs/release/java/index.html |
| Command-Based Programming | https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html |

### When to reference WPILib docs

- Any `SubsystemBase`, `CommandBase`, or `Command` usage
- Scheduler behavior, command composition (`Commands.sequence`, `.andThen`, `.alongWith`, etc.)
- Trigger and button binding (`Trigger`, `CommandXboxController`)
- `Timer`, `DriverStation`, `RobotController` utilities
- Sendable, SmartDashboard, or NetworkTables usage
- `PIDController`, `ProfiledPIDController`, `SimpleMotorFeedforward`, etc.
- Pose estimation, kinematics, odometry classes
- Any class from `edu.wpi.first.*`

---

## CTRE Phoenix 6

| Resource | URL |
|---|---|
| Phoenix 6 Docs | https://v6.docs.ctr-electronics.com/en/latest/index.html |
| Java API Reference | https://api.ctr-electronics.com/phoenix6/stable/java/ |

### When to reference CTRE docs

- `TalonFX` configuration, control modes, and status signals
- `CANcoder` configuration and signal reading
- `Pigeon2` IMU usage
- `TunerSwerveDrivetrain` and swerve module setup
- `StatusSignal`, `BaseStatusSignal.refreshAll()`, and signal latency compensation
- `MotionMagic`, `VelocityVoltage`, `PositionVoltage`, and other control request types
- CAN bus configuration and fault handling
- Any class from `com.ctre.phoenix6.*`

---

## Required Practice

1. **Before implementing any CTRE or WPILib feature**, fetch the relevant API doc page to confirm method signatures, required configuration steps, and deprecation status.
2. **Do not guess constructor arguments or configuration field names.** Look them up.
3. **Check for version-specific behavior.** This project targets WPILib 2026 and Phoenix 6 stable. Confirm that examples in the docs match these versions.
4. **Cross-reference the conceptual docs** (docs.wpilib.org / v6.docs.ctr-electronics.com) with the API reference (the index pages) — the conceptual docs explain *why*, the API reference confirms *how*.
