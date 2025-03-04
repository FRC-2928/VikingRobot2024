package frc.robot;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	public static Robot instance;
	public static final RobotContainer cont = RobotContainer.getInstance();

	public Robot() {
		super();
		ConduitApi.getInstance().configurePowerDistribution(Constants.CAN.Misc.pdh, ModuleType.kRev.value);

		switch(Constants.mode) {
		case REAL -> {
			Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
			Logger.addDataReceiver(new NT4Publisher());
		}

		case SIM -> {
			Logger.addDataReceiver(new NT4Publisher());
		}

		case REPLAY -> {
			this.setUseTiming(false); // Run as fast as possible
			final String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		}

		Logger.start();

		Robot.instance = this;

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotInit() {
		cont.drivetrain.limelight.setIMUMode(1);
		cont.elevator.setDefaultCommand();
		initializeSmartDashboard();
		
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev);
		cont.drivetrain.limelight.setRobotOrientation(cont.drivetrain.getEstimatedPosition().getRotation().getMeasure());
		Logger.recordOutput("RobotTriggers/CloseToReef", cont.driverOI.closeToReef);
		Logger.recordOutput("RobotTriggers/CloseToHP", cont.driverOI.closeToHP);
		Logger.recordOutput("RobotTriggers/CloseToProcessor", cont.driverOI.closeToProcessor);
	}

	// DISABLED //
	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		cont.drivetrain.limelight.setIMUMode(1);
	}

	@Override
	public void disabledPeriodic() {
		cont.drivetrain.disabledPeriodic();
		//get current selected routine
		//get pose from map (and default)
		//find differene from est and map pose
		String selected = SmartDashboard.getString("Autonomous Routine/selected", "none");
		Pose2d autoStartPose = Autonomous.getAutoStartingPose(selected, DriverStation.getAlliance().orElse(Alliance.Blue));
		if(selected != "none" && RobotContainer.getInstance().drivetrain.getEstimatedPosition() != null){
			Pose2d difference = RobotContainer.getInstance().drivetrain.est.getEstimatedPosition()
			.relativeTo(autoStartPose);
			if(difference != null){
				Logger.recordOutput("Drivetrain/Auto/differenceFromStartingPosition", Math.hypot(difference.getX(), difference.getY()));
			}
		}
		else{
			Logger.recordOutput("Drivetrain/Auto/differenceFromStartingPosition", 999d);
		}
		Logger.recordOutput("Drivetrain/Auto/selectedRoutine", selected);
		Logger.recordOutput("Drivetrain/Auto/startPose", autoStartPose);
		FieldObject2d startPose = RobotContainer.getInstance().drivetrain.field.getObject("StartPose");
		startPose.setPose(autoStartPose);
	}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
		cont.drivetrain.limelight.setIMUMode(2);
	}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		RobotContainer.getInstance().drivetrain.setDefaultCommand();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		RobotContainer.getInstance().drivetrain.setDefaultCommand();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}

	public void initializeSmartDashboard(){
		final SwerveModuleState[] states = RobotContainer.getInstance().drivetrain.currentModuleStates();
		SmartDashboard.putData("Swerve Drive", new Sendable() {
		@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");

				builder.addDoubleProperty("Front Left Angle", () -> states[0].angle.getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> states[0].speedMetersPerSecond, null);

				builder.addDoubleProperty("Front Right Angle", () -> states[1].angle.getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> states[1].speedMetersPerSecond, null);

				builder.addDoubleProperty("Back Left Angle", () -> states[2].angle.getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> states[2].speedMetersPerSecond, null);

				builder.addDoubleProperty("Back Right Angle", () -> states[3].angle.getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> states[3].speedMetersPerSecond, null);

				builder.addDoubleProperty("Robot Angle", () -> RobotContainer.getInstance().drivetrain.est.getEstimatedPosition().getRotation().getRadians(), null);
			}
		});
		final String selectedRoutine = SmartDashboard.getString("Autonomous Routine/selected", "none");
		FieldObject2d startPose = RobotContainer.getInstance().drivetrain.field.getObject("StartPose");
		if(Autonomous.autoMapBlue.containsKey(selectedRoutine)){
			startPose.setPose(Autonomous.getAutoStartingPose(selectedRoutine, DriverStation.getAlliance().orElse(Alliance.Blue)));
		}
		else{
			startPose.setPose(new Pose2d(0.0,0.0,new Rotation2d(0)));
		}
		SmartDashboard.putData("Field", RobotContainer.getInstance().drivetrain.field);
	}
	private boolean isInArray(String[] array,String check){
		for(int i=0; i<array.length;i++){
			if(array[i] == check){
				return true;
			}
		}
		return false;
	}
}
