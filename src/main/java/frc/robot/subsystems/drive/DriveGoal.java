package frc.robot.subsystems.drive;

/**
 * High-level intent for the drive subsystem, set by the Superstructure (future)
 * or directly by commands (current, pre-Superstructure).
 *
 * Safe default: TELEOP (driver joystick has control).
 */
public enum DriveGoal {
    /** Driver joystick controls the robot. Default state during teleop. */
    TELEOP,

    /** Wheels locked in X formation. Used for defense and end-of-match. */
    LOCK,

    /** Path follower (PathPlanner/BLine) controls the robot. Used during auto. */
    AUTONOMOUS
}
