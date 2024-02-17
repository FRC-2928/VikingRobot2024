package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
	public final Drivetrain drivetrain;
	public final DriverOI oi;

	public Rotation2d absoluteTarget = new Rotation2d();
	public double absoluteTargetMagnitude = 0.5;
	private final PIDController absoluteController = Constants.Drivetrain.absoluteRotationPID.createController();
	private final PIDController targetVerticalController = Constants.Drivetrain.targetVerticalControllerPID.createController();

	public JoystickDrive(final Drivetrain drivetrain, final DriverOI oi) {
		this.drivetrain = drivetrain;
		this.oi = oi;

		this.addRequirements(drivetrain);

		this.absoluteController.enableContinuousInput(-0.5, 0.5);

		// Set the rotation target angle equal to the pose
		this.absoluteTarget = this.drivetrain.getPose().getRotation();
	}

	@Override
	public void execute() {
		// final double mul = MathUtil.interpolate(1, 0.5, this.oi.slow.get());
		final double mul = 1;

		Translation2d linearVelocity;
		double omegaRadPerSec = 0;

		// Check if we're aligning the shooter with the target (rightTriggerAxis)
		boolean alignTriggerValue = this.oi.alignShooter.getAsBoolean();
		SmartDashboard.putBoolean("JoystickDrive/alignTriggerValue", alignTriggerValue);

		if (alignTriggerValue) {
			// Align robot with target	
			linearVelocity = getAlignTarget();
		} else {
			// 1. CONVERT JOYSTICK VALUES
			linearVelocity = getLinearVelocity(mul); // Meters per/sec
			omegaRadPerSec = getOmega(mul); // Radians per/sec		
		}	

		// 2 CONVERT TO CHASSIS SPEEDS	
		ChassisSpeeds desired = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omegaRadPerSec);

		// Compensate for wheel rotation while driving and rotating.
		if(Constants.Drivetrain.Flags.thetaCompensation) desired = this.drivetrain.compensate(desired);

		// 3. CONVERT FROM FIELD RELATIVE SPEED TO ROBOT RELATIVE CHASSIS SPEEDS
		if(Constants.Drivetrain.Flags.fod) desired = this.drivetrain.fieldOrientedDrive(desired);

		// 4. CONVERT CHASSIS SPEEDS TO MODULE SPEEDS
		// ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desired, 0.02);
		SwerveModuleState[] setpointStates = this.drivetrain.kinematics.toSwerveModuleStates(desired);

		// Set the required speed and angle of each wheel.
		this.drivetrain.setModuleStates(setpointStates);
	}

	/**
	 * Uses the Joystick Left Axis
	 * 
	 * @param mul - joystick input to slow the speed
	 * 
	 * @return the velocity in meters per/sec
	 */
	private Translation2d getLinearVelocity(final double mul) {
		
		// Get joystick inputs and apply deadbands
		double axial = MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
		double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);

		// Get the angle theta from the conversion of rectangular coordinates to polar coordinates
		final Rotation2d coordinatePlaneMoveDirection = Rotation2d.fromRadians(Math.atan2(axial, lateral));

		// Rotate this 90 degrees to align with the field direction
		final Rotation2d fieldPlaneMoveDirection = coordinatePlaneMoveDirection.minus(new Rotation2d(Math.PI / 2));

		// Calculate the move magnitude
		double magnitude = Math.hypot(lateral, axial);
		magnitude = MathUtil.clamp(magnitude, 0, 1); // Make it a unit value
		final double moveMagnitude = magnitude * magnitude; // convert position to velocity
		
		// Get the X and Y components of the velocity
		double xSpeed = Math.cos(fieldPlaneMoveDirection.getRadians()) * moveMagnitude * mul;
		double ySpeed = Math.sin(fieldPlaneMoveDirection.getRadians()) * moveMagnitude * mul;

		// Convert to meters per/sec
		double vxMetersPerSecond = xSpeed * Constants.Drivetrain.maxVelocityMetersPerSec;
		double vyMetersPerSecond = ySpeed * Constants.Drivetrain.maxVelocityMetersPerSec;
		SmartDashboard.putNumber("JoystickDrive/Linear Velocity X", vxMetersPerSecond);
		SmartDashboard.putNumber("JoystickDrive/Linear Velocity Y", vyMetersPerSecond);

		return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
	}

	/**
	 * Computes the omega radians per second required to move the
	 * robot pose to the new desired rotation angle. Omega radians per second
	 * will be zero if the target angle is aligned with the robot pose.
	 * 
	 * @param mul - joystick input to slow the speed
	 * 
	 * @return omega radians per second
	 */
	private double getOmega(final double mul) {
		
		double omega;
		if(Constants.Drivetrain.Flags.absoluteRotation) {
			// Joystick Right Axis
			final double rotX = this.oi.moveRotationX.get();
			final double rotY = this.oi.moveRotationY.get();

			// This will determine the rotation speed based on how far the joystick is moved.
			this.absoluteTargetMagnitude = Math.sqrt(rotX * rotX + rotY * rotY);
			SmartDashboard.putNumber("JoystickDrive/absoluteTargetMagnitude", absoluteTargetMagnitude);

			// Get a new rotation target if right joystick values are beyond the deadband.
			// Otherwise, we'll keep the old one.
			final boolean rotateRobot = this.absoluteTargetMagnitude > 0.5;
			if(rotateRobot) {
				Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(rotY, rotX));
				this.absoluteTarget = joystickAngle.minus(Rotation2d.fromRadians(Math.PI / 2));
			}
			SmartDashboard.putNumber("JoystickDrive/absoluteTarget", absoluteTarget.getDegrees());

			// Run a PID loop to adjust the angular rate of change as we approach the setpoint angle
			double measurement = Constants.mod(this.drivetrain.getPose().getRotation().getRotations(),1);
			double setpoint = this.absoluteTarget.getRotations();
			omega = MathUtil.clamp(this.absoluteController.calculate(measurement, setpoint), -0.5, 0.5);
			omega = MathUtil.applyDeadband(omega, rotateRobot ? 0.075 : 0.25); 

			this.absoluteTargetMagnitude = this.absoluteTargetMagnitude * 0.5 + 0.5;

			omega = omega * this.absoluteTargetMagnitude;
		} else {
			omega = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);
		}

		double omegaRadiansPerSecond = omega * Constants.Drivetrain.maxAngularVelocityRadPerSec * mul;
		SmartDashboard.putNumber("JoystickDrive/Omega Rad PerSec", omegaRadiansPerSecond);
		return omegaRadiansPerSecond;
	}

	/**
	 * Control the robot based on range from target
	 * 
	 * @return The required X and Y translation
	 */
	private Translation2d getAlignTarget() {

		double measurement = this.drivetrain.limelight.getTargetVerticalOffset();
		double xSpeed = this.targetVerticalController.calculate(measurement, 0);
		// xSpeed should end up between -1 and 1
		SmartDashboard.putNumber("JoystickDrive/Target align xSpeed", xSpeed);
		xSpeed = MathUtil.clamp(xSpeed, 0, 1); // Make it a unit value?

		double ySpeed = 0;
		double vxMetersPerSecond = xSpeed * Constants.Drivetrain.maxVelocityMetersPerSec;
		double vyMetersPerSecond = ySpeed * Constants.Drivetrain.maxVelocityMetersPerSec;
		SmartDashboard.putNumber("JoystickDrive/Align Target X", vxMetersPerSecond);
		SmartDashboard.putNumber("JoystickDrive/Align Target Y", vyMetersPerSecond);

		return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
	}
}
