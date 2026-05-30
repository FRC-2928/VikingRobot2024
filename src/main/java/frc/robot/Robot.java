package frc.robot;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.IntakeGround;
import frc.robot.commands.shooter.LookForNote;

public class Robot extends LoggedRobot {
	public static Robot instance;
	public static RobotContainer cont;
	public static Command commandToRun;
	public static boolean needToLookOtherWay;

	public RobotContainer container;

	private Command autonomousCommand;

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
		LiveWindow.disableAllTelemetry();

		Robot.instance = this;
		Robot.cont = new RobotContainer();

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev);
	}

	// DISABLED //
	@Override
	public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	private void resetPoseFromLimelight() {
		var rear = this.container.drivetrain.limelightRear;
		if (rear.hasValidTargets()) {
			Pose2d pose = rear.getPose2d();
			// Guard against all-zero default returned when no tags are actually resolved
			if (pose.getTranslation().getNorm() > 0.5) {
				this.container.drivetrain.resetPose(pose);
				Logger.recordOutput("Robot/OnEnable/Source", "Limelight-Rear");
				return;
			}
		}
		Logger.recordOutput("Robot/OnEnable/Source", "None");
	}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		if (DriverStation.isFMSAttached()) {
			this.container.getAutoStartPose().ifPresent(pose ->
				this.container.drivetrain.resetPose(Autonomous.getPoseForAlliance(pose))
			);
			Logger.recordOutput("Robot/OnEnable/Source", "FMS-AutoStart");
		} else {
			resetPoseFromLimelight();
		}

		this.container.shooter.io.retractAmpBar();

		// Get selected routine from the dashboard
		// this.autonomousCommand = this.container.getAutonomousCommand();

		// schedule the autonomous command (example)
		// if(this.autonomousCommand != null) {
		// 	this.autonomousCommand.schedule();
		// }
		Robot.commandToRun = new LookForNote(Units.Radians.of(Math.PI/4));
		Robot.needToLookOtherWay = true;
		Robot.commandToRun.schedule();
		//TODO: fix this whole thing
	}

	@Override
	public void autonomousPeriodic() {
		if (Robot.commandToRun != null) {
			// if (this.commandToRun.isFinished() && Robot.cont.drivetrain.limelightNote.hasValidTargets() && !commandHasFinished) {
			if (Robot.commandToRun.isFinished()) {
				//get new command to run, if we have one...
				// how do we know if we have one?
				//case 1: we have a target
				
				if (Robot.cont.drivetrain.limelightNote.hasValidTargets()) {
					Robot.commandToRun = new IntakeGround(true).withTimeout(4);
					Robot.needToLookOtherWay = false;
				} else if (Robot.needToLookOtherWay) {
					Robot.commandToRun = new LookForNote(Units.Radians.of(-Math.PI/2));
					Robot.needToLookOtherWay = false;
				} else {
					// no other options -- we're done
					Robot.commandToRun = null;
				}
				if(Robot.commandToRun != null){
					Robot.commandToRun.schedule();
				}
				//case 2:
			}
		} 
		Logger.recordOutput("Drivetrain/Auto/LimeLightHasValidTarget", Robot.cont.drivetrain.limelightNote.hasValidTargets());
	}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		if (!DriverStation.isFMSAttached()) {
			resetPoseFromLimelight();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
