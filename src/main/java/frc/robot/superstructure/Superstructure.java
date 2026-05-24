package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Single authority for all subsystem goals during a match.
 *
 * <p>Every 20ms:
 * <ol>
 *   <li>{@link GoalResolver} reads stored intents + DriverStation state → {@link RobotGoal}</li>
 *   <li>A {@link SuperstructureContext} is built from the goal (+ subsystem state snapshots in
 *       later phases)</li>
 *   <li>Each subsystem receives {@code applyGoal(ctx)} — the only place subsystems are told what
 *       to do</li>
 * </ol>
 *
 * <p>OI classes interact with the Superstructure via intent commands (e.g.,
 * {@link #setDriveIntentCommand}). Auto routines use {@link #setGoalCommand}.
 * Neither OI nor auto routines ever call subsystems directly.
 */
public class Superstructure extends SubsystemBase {

    private final DriveSubsystem drivetrain;
    private final GoalResolver resolver = new GoalResolver();

    public Superstructure(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    // -------------------------------------------------------------------------
    // Periodic loop
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        RobotGoal goal = resolver.resolve();
        SuperstructureContext ctx = new SuperstructureContext(goal);
        drivetrain.applyGoal(ctx);
    }

    // -------------------------------------------------------------------------
    // OI intent commands
    // -------------------------------------------------------------------------

    /**
     * Returns a command that pushes {@code intent} to the drive subsystem while active, then
     * reverts to the default ({@link DriveGoal#TELEOP}) when interrupted or finished.
     *
     * <p>Bind with {@code trigger.whileTrue(superstructure.setDriveIntentCommand(DriveGoal.LOCK))}.
     */
    public Command setDriveIntentCommand(DriveGoal intent) {
        return Commands.startEnd(
                () -> resolver.setDriveIntent(intent),
                () -> resolver.setDriveIntent(DriveGoal.TELEOP));
    }

    // -------------------------------------------------------------------------
    // Auto routine commands
    // -------------------------------------------------------------------------

    /**
     * Returns a command that overrides the resolver with the given goal for one cycle (runOnce).
     * The resolver's stored intents are updated to match, so the goal persists until OI or the
     * next setGoalCommand changes them.
     *
     * <p>Usage:
     * <pre>
     * Commands.sequence(
     *     superstructure.setGoalCommand(RobotGoal.autonomous()),
     *     pathCommand("pickupPath"),
     *     superstructure.setGoalCommand(RobotGoal.freeDrive())
     * )
     * </pre>
     */
    public Command setGoalCommand(RobotGoal goal) {
        return Commands.runOnce(() -> resolver.setGoal(goal));
    }
}
