package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;
import frc.robot.vision.Limelight;

public class Drivetrain extends SubsystemBase {
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;

	// Used to track odometry
	public final SwerveDriveOdometry pose;
	public final SwerveDrivePoseEstimator poseEstimator;
	private final Limelight limelight = new Limelight("limelight");

	private final SysIdRoutine sysId;
	
	// ----------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------
	public Drivetrain(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO) 
  	{	
		if (gyroIO == null) gyroIO = new GyroIOSim(this);
		this.gyroIO = gyroIO;
		resetGyro();

		modules[0] = new SwerveModule(flModuleIO, Place.FrontLeft);
		modules[1] = new SwerveModule(frModuleIO, Place.FrontRight);
		modules[2] = new SwerveModule(blModuleIO, Place.BackLeft);
		modules[3] = new SwerveModule(brModuleIO, Place.BackRight);

		poseEstimator = new SwerveDrivePoseEstimator( this.kinematics,
													getRobotAngle(),
													this.getModulePositions(),
													new Pose2d() 	
													);

		pose = new SwerveDriveOdometry( this.kinematics,
										getRobotAngle(),
										this.getModulePositions(),
										new Pose2d() 	
										);

		// Configure SysId
		sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
	}

	// ----------------------------------------------------------
    // Control Output
    // ----------------------------------------------------------

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param vxMetersPerSecond Speed of the robot in the x direction (forward).
	 * @param vyMetersPerSecond Speed of the robot in the y direction (sideways).
	 * @param omegaRadiansPerSecond Angular rate of the robot in radians per/sec
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drive(double vxMetersPerSecond, double vyMetersPerSecond, 
					  double omegaRadiansPerSecond, boolean fieldRelative) {
		
		SmartDashboard.putNumber("xSpeed", vxMetersPerSecond);
		SmartDashboard.putNumber("ySpeed", vyMetersPerSecond);
		SmartDashboard.putNumber("rotation", omegaRadiansPerSecond);

		var swerveModuleStates =
			kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, getPose().getRotation())
					: new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond));

		setModuleStates(swerveModuleStates);
	}

	/**
	 * Set the required speed and angle of each wheel.
	 * 
	 * @param states - required speed in meters per/sec
	 * 				 - angle in radians per/sec
	*/
	public void setModuleStates(final SwerveModuleState[] states) {
		// 5. DESATURATE WHEEL SPEEDS
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocityMetersPerSec);

		// 6. SET SPEED AND ANGLE FOR EACH WHEEL
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void setModuleStates(final SwerveModule.State state) { this.setModuleStates(state.states); }

	// Convert field-relative ChassisSpeeds to robot-relative ChassisSpeeds.
	public ChassisSpeeds fieldOrientedDrive(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, getPose().getRotation());
	}

	// Compensate for wheel rotation.  This prevents the robot 
	// from drifting to one side while driving and rotating
	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}
  
	public void resetGyro() {
		gyroIO.resetGyro();
	}
	
	// ----------------------------------------------------------
    // Sensor Input
    // ----------------------------------------------------------

	/**
	 * Current reported yaw of the Pigeon2 in degrees
	 * Constructs and returns a Rotation2d.
	 * Rotation is continuous through 360 degrees.
	 * 
	 * @return current angle of the robot
	 */
	@AutoLogOutput(key = "Robot/Rotation")
	public Rotation2d getRobotAngle() {
		return this.gyroInputs.yawPosition.unaryMinus();	
	}

	public double getAngularVelocity() {
		return gyroInputs.yawVelocityRadPerSec;
    }

	/**
	 * Takes the negative of the current angular value
	 * and converts given radians to rotations.
	 * 
	 * @return the continuous rotations and partial rotations
	 */
	public double getGyroRotations() {
		return getRobotAngle().getRotations();
	}

	public SwerveModule[] getSwerveModules() {return this.modules;}

	public SwerveDriveKinematics getKinematics() {return this.kinematics;}

	// ----------------------------------------------------------
    // Odometry
    // ----------------------------------------------------------
	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometryEstimator(Pose2d pose) {
		this.poseEstimator.resetPosition(
			getRobotAngle(),
			this.getModulePositions(),
			pose);
	}

	public void resetOdometry(Pose2d newPose) {
		this.pose.resetPosition(
			getRobotAngle(),
			this.getModulePositions(),
			newPose);
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Estimation")
	public Pose2d getPoseEstimation() {
		return this.poseEstimator.getEstimatedPosition();
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Pose")
	public Pose2d getPose() {
		return this.pose.getPoseMeters();
	}

	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::updateModulePosition).toArray(SwerveModulePosition[]::new);
	}

	@AutoLogOutput(key = "SwerveStates/Desired")
	public SwerveModuleState[] getModuleStates() {
		return Arrays.stream(this.modules).map(SwerveModule::getModuleState).toArray(SwerveModuleState[]::new);
	}

	/**
	 * Updates the pose with vision if close to current position.
	 */
	public void updatePoseEstimatorWithVision() {
		if(this.limelight.hasValidTargets()) {
			// distance from current pose to vision estimated pose
			double poseDifference = this.poseEstimator.getEstimatedPosition().getTranslation()
				.getDistance(this.limelight.getPose2d().getTranslation());
			SmartDashboard.putNumber("pose difference", poseDifference);

			if (poseDifference < 0.5) {
				this.poseEstimator.addVisionMeasurement(this.limelight.getPose2d(), Timer.getFPGATimestamp() - 0.3);
			}
		}
	}

	// ----------------------------------------------------------
    // Process Logic
    // ----------------------------------------------------------
	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		// Update the state of each swerve module
		for(final SwerveModule module : this.modules)
			module.update();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});		
		}
		
		// Update the odometry pose
		this.pose.update(getRobotAngle(), this.getModulePositions());
		this.poseEstimator.update(getRobotAngle(), this.getModulePositions());

		// Fuse odometry pose with vision data if we have it.
		updatePoseEstimatorWithVision();
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}
	  
	// ----------------------------------------------------------
    // Simulation
    // ----------------------------------------------------------
	public boolean getHasValidTargetsSim() {
		double heading = this.getPoseEstimation().getRotation().getDegrees();

		// if(DriverStation.getAlliance() == DriverStation.Alliance.Red) return heading < 75 || heading > -75;
		// else 
		return heading > 135 || heading < -135;
	}

}
