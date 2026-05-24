package frc.robot.superstructure;

/**
 * Immutable snapshot passed to every subsystem's {@code applyGoal()} each cycle.
 *
 * <p>Contains the current {@link RobotGoal} plus read-only state snapshots of all subsystems.
 * Subsystems use this to check interlocks against other subsystems without direct references.
 *
 * <p>Phase 2: only {@code goal} is present. Subsystem state fields will be added in Phase 3/4
 * as interlocks require them (e.g., {@code shooterState}, {@code climberState}).
 */
public record SuperstructureContext(RobotGoal goal) {}
