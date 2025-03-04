package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
	public final Drivetrain drivetrain;
	public final DriverOI oi;
	public double forMagnitude = 0.5;
	private double speedMultiplier;
		
		private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
			.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);
	
		public JoystickDrive(final Drivetrain drivetrain, double speedMultiplier) {
			this.drivetrain = drivetrain;
			this.oi = RobotContainer.getInstance().driverOI;
			this.speedMultiplier = speedMultiplier;
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

	@Override
	public void execute() {
		final ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds(), new Rotation2d(drivetrain.getFieldOrientedAngle()));
		drivetrain.control(robotOrientedSpeeds);
	}

	// Returns the Field-oriented ChassisSpeeds based on the joystick inputs
	public ChassisSpeeds speeds() {
		if(DriverStation.isAutonomous()) return new ChassisSpeeds();

		final Translation2d translation = this.translation();
		return new ChassisSpeeds(translation.getX() * speedMultiplier, translation.getY() * speedMultiplier, this.theta().in(Units.RadiansPerSecond) * speedMultiplier);
	}

	// Returns the translation (X and Y) component from the joystick
	private Translation2d translation() {
		// get inputs, apply deadbands
		double axial = this.oi.driveAxial.getAsDouble();
		double lateral = this.oi.driveLateral.getAsDouble();
		axial = -MathUtil.applyDeadband(axial, 0.25); // Negate b/c joystick Y is inverted from field X
		lateral = -MathUtil.applyDeadband(lateral, 0.25); // Negate b/c joystick X is inverted from field Y
		Logger.recordOutput("Drivetrain/JoystickDrive/Axial", axial);
		Logger.recordOutput("Drivetrain/JoystickDrive/Lateral", lateral);

		// Calculate the move magnitude
		final double magnitude = Math.hypot(axial, lateral); // get length and normalize
		final double desaturationFactor = Math.max(magnitude, 1.0); // guarantees the output is between -1 and 1

		// Convert to m/s
		final LinearVelocity vx = Constants.Drivetrain.maxVelocity.times(axial / desaturationFactor);
		final LinearVelocity vy = Constants.Drivetrain.maxVelocity.times(lateral / desaturationFactor);

		return new Translation2d(vx.in(Units.MetersPerSecond), vy.in(Units.MetersPerSecond));
	}

	// Returns the rotation component from the joystick
	private AngularVelocity theta() {
		double theta = 0;

		final String selectedDriveMode = RobotContainer.getInstance().getDriveMode();
		if("Swerve Drive".equals(selectedDriveMode)) {
			theta = -this.oi.driveFORX.getAsDouble();
			theta = MathUtil.applyDeadband(theta, 0.075); // Negate this b/c joystick X is inverted from robot rotation
		} else {
			// Joystick Right Axis
			final double rotX = this.oi.driveFORX.getAsDouble();
			final double rotY = this.oi.driveFORY.getAsDouble();

			// This will determine the rotation speed based on how far the joystick is moved.
			this.forMagnitude = Math.hypot(rotX, rotY);
			Logger.recordOutput("JoystickDrive/AbsoluteRotation/Magnitude", this.forMagnitude);

			// Get a new rotation target if right joystick values are beyond the deadband.
			// Otherwise, we'll keep the old one.
			final boolean doRotateRobot = this.forMagnitude > 0.5;
			Angle forTarget = Units.Radians.zero();
			if (doRotateRobot) {
				forTarget = Units.Radians.of(-Math.atan2(rotX, rotY));
				Logger.recordOutput("JoystickDrive/AbsoluteRotation/Target", forTarget);
				final double measurement = this.drivetrain.getFieldOrientedAngle().in(Units.Rotations);
				final double setpoint = forTarget.in(Units.Rotations);
				
				theta = MathUtil.applyDeadband(
					this.absoluteController.calculate(measurement, setpoint),
					0.075
				);
			}

			this.forMagnitude = this.forMagnitude * 0.5 + 0.5;
		}

		return Constants.Drivetrain.maxAngularVelocity.times(theta * this.forMagnitude);
	}
}
