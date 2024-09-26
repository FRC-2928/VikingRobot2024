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

	public ShootSpeaker(final boolean isTeleopCommand) { this(isTeleopCommand, Constants.Shooter.readyShootRear, 0); }

	public ShootSpeaker(final boolean isTeleopCommand, final double timeout) {
		this(isTeleopCommand, Constants.Shooter.readyShootRear, timeout);
	}

	public ShootSpeaker(final boolean triggerFire, final Angle startAngle, final double timeout) {
		this.addRequirements(Robot.cont.shooter);
		this.mIsTeleopCommand = isTeleopCommand;
		this.rearAngle = startAngle;
		this.timeout = timeout;
	}

	// indicates if this ShootSpeaker Command was instantiated for teleop
	public final boolean mIsTeleopCommand;
	public final Angle rearAngle;
	public final double timeout;

	private double firedTime;
	private double startTime;
	private final SimpleMotorFeedforward targetRotationFeedforward = new SimpleMotorFeedforward(0, 5);

	@Override
	public void initialize() {
		this.firedTime = -1;
		this.startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		// indicates if the robot is facing forward from a robot-oriented perspective (rather than field-oriented)
		// note that we use cosine < 0 here since field-forward is away from our driver-station (which is where the speaker is)
		final boolean facingForward = Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getCos() < 0;
		// indicates if the shooter is currently facing (robot-oriented) forward
		final boolean isShooterForward = Robot.cont.shooter.inputs.angle.in(Units.Degrees) - 90 < 0;
		Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
		Robot.cont.drivetrain.limelightShooter.setPipeline(facingForward ? 0 : 1);

		if(facingForward && isShooterForward) {
			final boolean shooterLLSees = Robot.cont.drivetrain.limelightShooter.hasValidTargets();
			final boolean rearLLSees = Robot.cont.drivetrain.limelightRear.hasValidTargets();
			final boolean flywheelAtSpeed =
				Robot.cont.shooter.inputs.flywheelSpeedA.in(Units.RotationsPerSecond)
					>= Tuning.flywheelVelocityThreshold.get();
			final boolean pivotVelocityWithinThreshold =
				Math.abs(Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond))
				< Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);
			// we should fire only if the driver wants to fire or the command doesn't require input (i.e., during auto)
			final boolean demandFire = Robot.cont.driverOI.intake.getAsBoolean() || !this.mIsTeleopCommand;
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
			Logger.recordOutput("Shooter/ShootSpeaker/FlywheelSpeed", flywheelAtSpeed);
			Logger.recordOutput("Shooter/ShootSpeaker/PivotVelocity", pivotVelocityWithinThreshold);
			Logger
				.recordOutput(
					"Shooter/ShootSpeaker/PivotVelocityDifference",
					Math.abs(Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond))
					//- Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond)
				);
			Logger.recordOutput("Shooter/ShootSpeaker/DemandFire", demandFire);
			Logger.recordOutput("Shooter/ShootSpeaker/OverrideShoot", overrideShoot);
			Logger.recordOutput("Shooter/ShootSpeaker/FiredTime", this.firedTime != -1);

			if(overrideShoot && demandFire) {
				Robot.cont.shooter.io.runFeeder(Demand.Forward);
				if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
			}

			if(shooterLLSees) {
				// pitch offset -- we use horizontal offset because the limelight is mounted sideways
				final Angle po = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
				// yaw offset -- using vertical offset because the limelight is mounted sideways
				final Angle yo = Robot.cont.drivetrain.limelightShooter
					.getTargetVerticalOffset()
					.times(isShooterForward ? 1 : -1);

				Logger.recordOutput("Shooter/ShootSpeaker/tx", po.in(Units.Degrees));
				Logger.recordOutput("Shooter/ShootSpeaker/ty", yo.in(Units.Degrees));

				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/ShooterAlign",
						this.targetRotationFeedforward.calculate(yo.in(Units.Rotations))
					);

				// The intent of this next series of calls is to help align the robot for a shot on goal, the logic is:
				// 1. obtain the joystickSpeeds from the drivetrain (NOTE: these may not be accurate as they are updated in Drivetrain periodic)
				//	  it's also worth noting this is a bunch of leaky abstractions...
				// 2. remove the rotational components from the obtained joystickSpeeds
				// 3. using the yaw offset, compute a feedforward output (clamped to 0.125 speed) and make a new ChassisSpeeds with it
				// 3a. Pass the rotation-only ChassisSpeeds to Drivetrain's rod method to transform it into field-relative speeds
				// 4. Combine the translation-only and rotation-only speeds into a single ChassisSpeeds object
				// 5. Finally, pass the combined ChassisSpeeds to the drivetrain for control
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

				boolean isPivotPosThersholdMet = (Math.abs(po.in(Units.Degrees)) >= Tuning.shootSpeakerPivotThreshold.get());
				boolean hasShooterFired = (this.firedTime == -1 ? false : true);
				// assuming this is "have we moved enough to start aiming"...
				// this is _likely_ why the override isn't working -- this condition is checked before the override is considered
				// we should probably move the override logic to an initial state check...
				if(isPivotPosThersholdMet && !hasShooterFired) {
					// rotate (presumably, set the shooter setpoint) to the current angle + the calculated pitch-based ff
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
					// if the flywheel is at speed, the pivot is in position, and the yaw offset is in range...
					// OR... shoot override is requested...
					// AND... demandFire...
					// OR... we simply haven't fired yet..
					if(
						(((flywheelAtSpeed && pivotVelocityWithinThreshold && yo.in(Units.Degrees) < 10) || overrideShoot) && demandFire)
							|| this.firedTime != -1
					) {
						// run the feeder and set the fired time to now
						Robot.cont.shooter.io.runFeeder(Demand.Forward);
						if(this.firedTime == -1) this.firedTime = Timer.getFPGATimestamp();
					}

					//Robot.cont.shooter.io.rotate(Units.Rotations.of(Robot.cont.shooter.inputs.angle.in(Units.Rotations)));
				}
			} else if(!facingForward) {
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

				// same auto-align as the previous case, but with the rear limelight's offsets...
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
				Robot.cont.shooter.io.rotate(this.rearAngle);
			} else {
				// shooter limelight doesn't have a target but we're facing the speaker -- rotate into position
				// and begin target acquisition sequence
				Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
				Robot.cont.shooter.io.rotate(Constants.Shooter.readyShootFront);
			}
		} else {
			// we're not facing the speaker or the shooter is not facing the right way...
			// either way, rotate the shooter into position
			Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
			Robot.cont.shooter.io.rotate(facingForward ? Constants.Shooter.readyShootFront : this.rearAngle);
		}

		// timeout mechanism 
		if(this.timeout > 0 && Timer.getFPGATimestamp() > this.startTime + this.timeout) {
			Robot.cont.shooter.io.runFeeder(Demand.Forward);
			if(this.firedTime == -1) this.firedTime = Timer.getFPGATimestamp();
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
		return this.firedTime != -1 && Timer.getFPGATimestamp() - this.firedTime >= Constants.Shooter.fireTimeout;
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
