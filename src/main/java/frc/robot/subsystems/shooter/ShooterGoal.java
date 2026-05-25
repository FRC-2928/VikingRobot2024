package frc.robot.subsystems.shooter;

/**
 * What the Superstructure is asking the shooter/intake subsystem to do this cycle.
 *
 * <p>Safe default is {@link #HOME}. All other goals represent active behavior.
 * Intake is part of this subsystem — there is no separate IntakeGoal.
 */
public enum ShooterGoal {
    /** Pivot at drive-safe angle, flywheels off, feeder stopped. Safe default. */
    HOME,

    /** Pivot at ground intake angle, intake roller running, flywheels reversing note in. */
    INTAKE,

    /** Limelight-based pivot aim, flywheels at speaker velocity, fires when aligned. */
    SHOOT_SPEAKER,

    /** Pivot at fixed sub-station angle, flywheels at speaker velocity, fires when ready. */
    SHOOT_FIXED,

    /** Amp bar extended, pivot at amp angle, flywheels at amp power, fires when ready. */
    AMP,

    /** Pivot at ferry angle, flywheels at speaker velocity, fires when ready. */
    FERRY
}
