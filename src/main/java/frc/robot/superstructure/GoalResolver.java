package frc.robot.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.climber.ClimberGoal;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.shooter.ShooterGoal;

/**
 * Maps OI intents to a concrete {@link RobotGoal} each cycle.
 *
 * <p>This class plays two roles described in CLAUDE.md:
 * <ul>
 *   <li><b>IntentStore</b> — stores per-subsystem intents set directly by OI triggers.</li>
 *   <li><b>GoalResolver</b> — combines stored intents with DriverStation state to produce the
 *       final {@link RobotGoal} for the cycle.</li>
 * </ul>
 *
 * <p>OI triggers set intents directly via {@link #setDriveIntent} / {@link #setShooterIntent}.
 * Auto routines use {@link #setGoal} to override all intents at once.
 */
public class GoalResolver {

    // Per-subsystem intent storage — default to safe values
    private DriveGoal   driveIntent   = DriveGoal.TELEOP;
    private ShooterGoal shooterIntent = ShooterGoal.HOME;
    private ClimberGoal climberIntent = ClimberGoal.IDLE;

    // -------------------------------------------------------------------------
    // Intent setters (called by Superstructure on behalf of OI)
    // -------------------------------------------------------------------------

    public void setDriveIntent(DriveGoal intent) {
        driveIntent = intent;
    }

    public void setShooterIntent(ShooterGoal intent) {
        shooterIntent = intent;
    }

    public void setClimberIntent(ClimberGoal intent) {
        climberIntent = intent;
    }

    /**
     * Overrides all intents to match the given goal. Used by auto routines via
     * {@link Superstructure#setGoalCommand}.
     */
    public void setGoal(RobotGoal goal) {
        driveIntent   = goal.drive();
        shooterIntent = goal.shooter();
        climberIntent = goal.climber();
    }

    // -------------------------------------------------------------------------
    // Resolution
    // -------------------------------------------------------------------------

    /**
     * Resolves stored intents + DriverStation state into the final goal for this cycle.
     *
     * <p>DriverStation autonomous mode forces {@link DriveGoal#AUTONOMOUS} regardless of intent,
     * so the path follower always has authority during auto.
     *
     * <p>Coordination rules (applied before building the goal):
     * <ul>
     *   <li>Climber DEPLOY → shooter is forced to HOME. The climber interlock in
     *       {@link frc.robot.subsystems.Climber#applyGoal} then holds it at IDLE until
     *       the shooter actually reaches a safe angle.</li>
     *   <li>Shooter non-HOME → climber is forced to RETRACT. The shooter interlock in
     *       {@link frc.robot.subsystems.Shooter#applyGoal} then holds it at HOME until
     *       the climber is actually fully retracted.</li>
     * </ul>
     */
    public RobotGoal resolve() {
        final DriveGoal drive = DriverStation.isAutonomous() ? DriveGoal.AUTONOMOUS : driveIntent;

        // Coordination: climber deploying forces shooter to HOME first.
        ShooterGoal shooter = (climberIntent == ClimberGoal.DEPLOY)
                ? ShooterGoal.HOME
                : shooterIntent;

        // Coordination: shooter moving away from HOME forces climber to RETRACT first.
        // Only applies when the climber is not already deploying (that case is handled above).
        ClimberGoal climber = (shooter != ShooterGoal.HOME && climberIntent != ClimberGoal.DEPLOY)
                ? ClimberGoal.RETRACT
                : climberIntent;

        return RobotGoal.builder()
                .withDrive(drive)
                .withShooter(shooter)
                .withClimber(climber)
                .build();
    }
}
