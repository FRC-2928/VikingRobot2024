package frc.robot.superstructure;

import frc.robot.subsystems.Shooter.ShooterState;

/**
 * Immutable snapshot passed to every subsystem's {@code applyGoal()} each cycle.
 *
 * <p>Contains the current {@link RobotGoal} plus read-only state snapshots of all subsystems.
 * Subsystems use this to check interlocks against other subsystems without direct references.
 *
 * <p>Phase 3: {@code shooterState} added for Phase 4 climber interlock
 * ({@code ctx.shooterState().atSafeAngle()}).
 */
public record SuperstructureContext(RobotGoal goal, ShooterState shooterState) {

    /** Convenience constructor for cycles where no shooter state is needed yet. */
    public SuperstructureContext(RobotGoal goal) {
        this(goal, new ShooterState(false, false, true));
    }
}
