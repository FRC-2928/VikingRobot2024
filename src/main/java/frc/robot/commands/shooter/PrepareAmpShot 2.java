package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class PrepareAmpShot extends Command {
	public PrepareAmpShot() { this.addRequirements(Robot.cont.shooter, Robot.cont.drivetrain); }

	@Override
	public void initialize() {
		Robot.cont.shooter.io.extendAmpBar();
		Robot.cont.shooter.io.rotate(Units.Degrees.of(10));
	}

	@Override
	public void execute() {
		// just allow drive...
		Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
	}

	@Override
	public void end(final boolean interrupted) {
		// no-op
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
