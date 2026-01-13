package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;

public class FinishAmpShot extends Command {
	public FinishAmpShot() { this.addRequirements(Robot.cont.shooter, Robot.cont.drivetrain); }

	@Override
	public void initialize() {
		// retract the amp deflector and rotate to a safe position
		Robot.cont.shooter.io.retractAmpBar();
		Robot.cont.shooter.io.rotate(Units.Degrees.of(10));
	}

	@Override
	public void execute() {
		Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
