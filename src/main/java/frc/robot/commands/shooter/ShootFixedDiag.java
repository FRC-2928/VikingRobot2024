package frc.robot.commands.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootFixedDiag extends Command {
	public ShootFixedDiag(final boolean triggerFire) { this(triggerFire, 0); }

	public ShootFixedDiag(final boolean triggerFire, final double timeout) {
		this(() -> Units.Degrees.of(Tuning.subAngle.get() - 2), triggerFire, timeout);
	}

	public ShootFixedDiag(final Supplier<Angle> angle, final boolean triggerFire, final double timeout) {
		this.addRequirements(Robot.cont.shooter);
		this.angle = angle;
		this.triggerFire = triggerFire;
		this.timeout = timeout;
	}

	public final Supplier<Angle> angle;
	public final boolean triggerFire;
	public final double timeout;

	private double fired;
	private double startTime;

	@Override
	public void initialize() {
		this.fired = -1;
		this.startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		final Angle angle = this.angle.get();
		Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
		Robot.cont.shooter.io.rotate(angle);

		final boolean pivotAngle = Math
			.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - angle.in(Units.Degrees)) < 1.25;
		final boolean flywheelSpeed = Robot.cont.shooter.inputs.flywheelSpeedA
			.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();
		final boolean pivotVelocity = Math
			.abs(
				Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond)
			) < Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);
		Logger
			.recordOutput(
				"Shooter/ShootSpeaker/PivotAngle",
				Math.abs(Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset().in(Units.Degrees)) < 1.25
			);
		Logger.recordOutput("Shooter/ShootSpeaker/FlywheelSpeed", flywheelSpeed);
		Logger.recordOutput("Shooter/ShootSpeaker/PivotVelocity", pivotVelocity);
		Logger
			.recordOutput(
				"Shooter/ShootSpeaker/PivotVelocityDifference",
				Math.abs(Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond))
				//- Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond)
			);
		Logger.recordOutput("Shooter/ShootSpeaker/Fired", this.fired != -1);

		if(
			(pivotAngle && flywheelSpeed && pivotVelocity)
				|| this.fired != -1
				|| (this.timeout > 0 && Timer.getFPGATimestamp() > this.startTime + this.timeout)
		) {
			Robot.cont.shooter.io.runFeeder(Demand.Forward);
			if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public boolean isFinished() {
		return this.fired != -1 && Timer.getFPGATimestamp() - this.fired >= Constants.Shooter.fireTimeout;
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
