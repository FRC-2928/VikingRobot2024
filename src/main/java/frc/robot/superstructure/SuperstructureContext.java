package frc.robot.superstructure;

import frc.robot.subsystems.Climber.ClimberState;
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
public record SuperstructureContext(RobotGoal goal, ShooterState shooterState, ClimberState climberState) {

    /** Convenience constructor for cycles where no climber state is needed. */
    public SuperstructureContext(RobotGoal goal, ShooterState shooterState) {
        this(goal, shooterState, new ClimberState(0, false, false, false));
    }

    /** Convenience constructor for cycles where no subsystem state is needed yet. */
    public SuperstructureContext(RobotGoal goal) {
        this(goal, new ShooterState(false, false, true), new ClimberState(0, false, false, false));
    }
}
