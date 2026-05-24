package frc.robot.superstructure;

import frc.robot.subsystems.drive.DriveGoal;

/**
 * Immutable snapshot of what every subsystem should be doing this cycle.
 *
 * <p>Build via {@link #builder()} — all fields default to their subsystem-safe defaults.
 * Only specify what needs to be active:
 *
 * <pre>
 * RobotGoal goal = RobotGoal.builder()
 *     .withDrive(DriveGoal.LOCK)
 *     .build();
 * </pre>
 *
 * <p>Derive from an existing goal via {@link #modify()}:
 *
 * <pre>
 * RobotGoal autoShoot = RobotGoal.shootAtHub()
 *     .modify()
 *     .withDrive(DriveGoal.AUTONOMOUS)
 *     .build();
 * </pre>
 */
public record RobotGoal(DriveGoal drive) {

    // -------------------------------------------------------------------------
    // Factory methods
    // -------------------------------------------------------------------------

    /** Driver has full joystick control. All subsystems at safe idle. */
    public static RobotGoal freeDrive() {
        return builder().build();
    }

    /** Wheels locked in X-formation. Used for defense or end-of-match. */
    public static RobotGoal lockWheels() {
        return builder().withDrive(DriveGoal.LOCK).build();
    }

    /** Path follower controls the drive. Used during auto routines. */
    public static RobotGoal autonomous() {
        return builder().withDrive(DriveGoal.AUTONOMOUS).build();
    }

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    /** Returns a new {@link Builder} with all fields set to safe defaults. */
    public static Builder builder() {
        return new Builder();
    }

    /** Returns a {@link Builder} pre-populated from this goal for easy derivation. */
    public Builder modify() {
        return new Builder(this);
    }

    public static class Builder {
        // Safe defaults: subsystems idle, driver has control
        private DriveGoal drive = DriveGoal.TELEOP;

        private Builder() {}

        private Builder(RobotGoal base) {
            this.drive = base.drive;
        }

        public Builder withDrive(DriveGoal drive) {
            this.drive = drive;
            return this;
        }

        public RobotGoal build() {
            return new RobotGoal(drive);
        }
    }
}
