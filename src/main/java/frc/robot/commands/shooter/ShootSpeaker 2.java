package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootSpeaker extends Command {
	private static final PIDController pitch = new PIDController(0.75, 0, 0);

	static {
		SmartDashboard.putData(ShootSpeaker.pitch);
	}

	public ShootSpeaker(final boolean triggerFire) { this(triggerFire, Constants.Shooter.readyShootRear, 0); }

	public ShootSpeaker(final boolean triggerFire, final double timeout) {
		this(triggerFire, Constants.Shooter.readyShootRear, timeout);
	}

	public ShootSpeaker(final boolean triggerFire, final Angle startAngle, final double timeout) {
		this.addRequirements(Robot.cont.shooter);
		this.triggerFire = triggerFire;
		this.rearAngle = startAngle;
		this.timeout = timeout;
	}

	public final boolean triggerFire;
	public final Angle rearAngle;
	public final double timeout;

	private double fired;
	private double startTime;
	private final SimpleMotorFeedforward targetRotationFeedforward = new SimpleMotorFeedforward(0, 5);

	@Override
	public void initialize() {
		this.fired = -1;
		this.startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		final boolean forward = Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getCos() < 0;
		final boolean current = Robot.cont.shooter.inputs.angle.in(Units.Degrees) - 90 < 0;
		Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
		Robot.cont.drivetrain.limelightShooter.setPipeline(forward ? 0 : 1);

		if(forward == current) {
			final boolean shooterLLSees = Robot.cont.drivetrain.limelightShooter.hasValidTargets();
			final boolean rearLLSees = Robot.cont.drivetrain.limelightRear.hasValidTargets();
			final boolean flywheelSpeed = Robot.cont.shooter.inputs.flywheelSpeedA
				.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();
			final boolean pivotVelocity = Math
				.abs(
					Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond)
				) < Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);
			final boolean demandFire = Robot.cont.driverOI.intake.getAsBoolean() || !this.triggerFire;
			final boolean overrideShoot = Robot.cont.operatorOI.overrideShoot.getAsBoolean();

			Logger.recordOutput("Shooter/ShootSpeaker/ShooterLLSees", shooterLLSees);
			Logger.recordOutput("Shooter/ShootSpeaker/RearLLSees", rearLLSees);
			Logger
				.recordOutput(
					"Shooter/ShootSpeaker/PivotAngle",
					Math
						.abs(
							Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset().in(Units.Degrees)
						) < 1.25
				);
			Logger.recordOutput("Shooter/ShootSpeaker/FlywheelSpeed", flywheelSpeed);
			Logger.recordOutput("Shooter/ShootSpeaker/PivotVelocity", pivotVelocity);
			Logger
				.recordOutput(
					"Shooter/ShootSpeaker/PivotVelocityDifference",
					Math.abs(Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond))
					//- Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond)
				);
			Logger.recordOutput("Shooter/ShootSpeaker/DemandFire", demandFire);
			Logger.recordOutput("Shooter/ShootSpeaker/OverrideShoot", overrideShoot);
			Logger.recordOutput("Shooter/ShootSpeaker/Fired", this.fired != -1);

			if(overrideShoot && demandFire) {
				Robot.cont.shooter.io.runFeeder(Demand.Forward);
				if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
			}

			if(shooterLLSees) {
				final Angle po = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
				final Angle yo = Robot.cont.drivetrain.limelightShooter
					.getTargetVerticalOffset()
					.times(forward ? 1 : -1);

				Logger.recordOutput("Shooter/ShootSpeaker/tx", po.in(Units.Degrees));
				Logger.recordOutput("Shooter/ShootSpeaker/ty", yo.in(Units.Degrees));

				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/ShooterAlign",
						this.targetRotationFeedforward.calculate(yo.in(Units.Rotations))
					);

				Robot.cont.drivetrain
					.control(
						this
							.norot(Robot.cont.drivetrain.joystickSpeeds)
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											MathUtil
												.clamp(
													this.targetRotationFeedforward.calculate(yo.in(Units.Rotations)),
													-0.125,
													0.125
												)
										)
									)
							)
					);

				if(Math.abs(po.in(Units.Degrees)) >= Tuning.shootSpeakerPivotThreshold.get() && this.fired == -1) {
					Robot.cont.shooter.io
						.rotate(
							Units.Rotations
								.of(
									Robot.cont.shooter.inputs.angle.in(Units.Rotations)
										+ ShootSpeaker.pitch
											.calculate(
												this.pow(po.in(Units.Rotations), Tuning.shootSpeakerExponent.get())
											)
								)
						);
				} else {
					if(
						(((flywheelSpeed && pivotVelocity && yo.in(Units.Degrees) < 10) || overrideShoot) && demandFire)
							|| this.fired != -1
					) {
						Robot.cont.shooter.io.runFeeder(Demand.Forward);
						if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
					}

					//Robot.cont.shooter.io.rotate(Units.Rotations.of(Robot.cont.shooter.inputs.angle.in(Units.Rotations)));
				}
			} else if(!forward) {
				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/RearAlign",
						this.targetRotationFeedforward
							.calculate(
								Robot.cont.drivetrain.limelightRear.getTargetHorizontalOffset().in(Units.Rotations)
							)
					);

				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/txr",
						Robot.cont.drivetrain.limelightRear.getTargetHorizontalOffset().in(Units.Degrees)
					);

				Robot.cont.drivetrain
					.control(
						this
							.norot(Robot.cont.drivetrain.joystickSpeeds)
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											this.targetRotationFeedforward
												.calculate(
													Robot.cont.drivetrain.limelightRear
														.getTargetHorizontalOffset()
														.in(Units.Rotations)
												)
										)
									)
							)
					);
				Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
			} else {
				Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
				Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
			}
		} else {
			Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
			Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
		}

		if(this.timeout > 0 && Timer.getFPGATimestamp() > this.startTime + this.timeout) {
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

	private ChassisSpeeds norot(final ChassisSpeeds speeds) {
		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
	}

	private double pow(final double base, final double exp) {
		return Math.copySign(Math.pow(Math.abs(base), exp), base);
	}
}
