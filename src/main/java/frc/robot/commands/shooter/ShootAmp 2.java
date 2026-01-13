package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootAmp extends Command {
	public ShootAmp() { this.addRequirements(Robot.cont.shooter, Robot.cont.drivetrain); }

	private boolean fired;

	@Override
	public void initialize() { this.fired = false; }

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(Units.Degrees.of(Tuning.ampAngle.get()));
		Robot.cont.shooter.io.runFlywheels(Tuning.ampPower.get());

		Robot.cont.drivetrain
			.control(
				Robot.cont.drivetrain.joystickSpeeds.plus(Robot.cont.drivetrain.rod(new ChassisSpeeds(0, 0, 0))).div(2)
			);

		if(
			(Math.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Tuning.ampAngle.get()) <= 20.5
				&& Robot.cont.driverOI.intake.getAsBoolean()) || this.fired
		) {
			Robot.cont.shooter.io.runFeeder(Demand.Forward);
			this.fired = true;
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
