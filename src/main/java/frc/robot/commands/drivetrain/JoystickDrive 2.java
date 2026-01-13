package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
	public JoystickDrive(final Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		this.addRequirements(this.drivetrain);

		this.absoluteController.enableContinuousInput(-0.5, 0.5);
	}

	public static SendableChooser<String> createDriveModeChooser() {
		final SendableChooser<String> chooser = new SendableChooser<>();
		chooser.addOption("Swerve Drive", "Swerve Drive");
		chooser.addOption("Field Oriented", "Field Oriented");
		chooser.setDefaultOption("Swerve Drive", "Swerve Drive");
		return chooser;
	}

	public final Drivetrain drivetrain;
	public final DriverOI oi = Robot.cont.driverOI;

	public Angle forTarget = Units.Radians.zero();
	public double forMagnitude = 0.5;
	private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
		.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);

	// no execute method, drivetrain handles that

	// this is a separate method so that drivetrain can call it to get base speeds to modify
	public ChassisSpeeds speeds() {
		if(DriverStation.isAutonomous()) return new ChassisSpeeds();

		final Translation2d translation = this.translation();
		return new ChassisSpeeds(translation.getX(), translation.getY(), this.theta().in(Units.RadiansPerSecond));
	}

	private Translation2d translation() {
		// get inputs, apply deadbands
		final double axial = MathUtil.applyDeadband(this.oi.driveAxial.get(), 0.1);
		final double lateral = MathUtil.applyDeadband(this.oi.driveLateral.get(), 0.1);

		// cartesian -> polar
		final Rotation2d direction = Rotation2d.fromRadians(Math.atan2(lateral, axial)); // why?

		// Calculate the move magnitude
		final double magnitude = Math.pow(MathUtil.clamp(Math.hypot(axial, lateral), 0, 1), 2); // get length and normalize

		final double dx = Math.cos(direction.getRadians()) * magnitude;
		final double dy = Math.sin(direction.getRadians()) * magnitude;

		// Convert to m/s
		final LinearVelocity vx = Constants.Drivetrain.maxVelocity.times(dx);
		final LinearVelocity vy = Constants.Drivetrain.maxVelocity.times(dy);

		return new Translation2d(vx.in(Units.MetersPerSecond), vy.in(Units.MetersPerSecond));
	}

	private AngularVelocity theta() {
		final double theta;

		final String selectedDriveMode = Robot.cont.getDriveMode();
		if("Swerve Drive".equals(selectedDriveMode)) {
			theta = MathUtil.applyDeadband(this.oi.driveFORX.get(), 0.075);
		} else {
			// Joystick Right Axis
			final double rotX = this.oi.driveFORX.get();
			final double rotY = this.oi.driveFORY.get();

			// This will determine the rotation speed based on how far the joystick is moved.
			this.forMagnitude = Math.hypot(rotX, rotY);
			Logger.recordOutput("JoystickDrive/AbsoluteRotation/Magnitude", this.forMagnitude);

			// Get a new rotation target if right joystick values are beyond the deadband.
			// Otherwise, we'll keep the old one.
			final boolean rotateRobot = this.forMagnitude > 0.5;
			if(rotateRobot) this.forTarget = Units.Radians.of(-Math.atan2(rotX, rotY));
			Logger.recordOutput("JoystickDrive/AbsoluteRotation/Target", this.forTarget);

			this.forMagnitude = this.forMagnitude * 0.5 + 0.5;

			final double measurement = this.drivetrain.est.getEstimatedPosition().getRotation().getRotations();
			final double setpoint = this.forTarget.in(Units.Rotations);

			theta = MathUtil
				.applyDeadband(
					-(this.absoluteController.calculate(measurement, setpoint)), // todo: determine whether this - is ok
					0.075
				);
		}

		return Constants.Drivetrain.maxAngularVelocity.times(theta * this.forMagnitude);
	}
}
