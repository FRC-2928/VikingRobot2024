package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.*;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.commands.shooter.*;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"[comp] Five Note",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					Autonomous.setInitialPose("MiddleFiveNote.1"),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.path("MiddleFiveNote.1"),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("MiddleFiveNote.3"),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4),
					new ReadyShooter(Constants.Shooter.readyIntake),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("MiddleFiveNote.4"),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					new ReadyShooter(Constants.Shooter.readyIntake),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("MiddleFiveNote.5"),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4),
					Autonomous.path("MiddleFiveNote.5"),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("MiddleFiveNote.6"),
					Autonomous.path("MiddleFiveNote.6"),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[testing] Five Note Middle",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					Autonomous.setInitialPose("5Note.1"),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.path("5Note.1"),
					new IntakeGround().withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.dynamic("5Note.4")
						// AutonomousRoutines.choreo(Choreo.getTrajectory("5Note.3"))
					),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4),
					new IntakeGround().withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.dynamic("5Note.6")
						// AutonomousRoutines.choreo(Choreo.getTrajectory("5Note.5"))
					),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					new IntakeGround().withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.path("5Note.7")
					),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					Autonomous.path("5Note.8"),
					new IntakeGround().withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.path("5Note.9")
					),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[testing] Source Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("SourceSideCenterNote.1"),
					new ReadyShooter(Constants.Shooter.readyShootRear),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.path("SourceSideCenterNote.1"),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("SourceSideCenterNote.2"),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.path("SourceSideCenterNote.2")
					),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[comp] Drive",
				new SequentialCommandGroup(new ReadyShooter(Constants.Shooter.readyShootRear), new DriveTime(1))
			);

		chooser
			.addOption(
				"[comp] Shoot/Drive",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					new ShootSpeaker(false),
					new DriveTime(1)
				)
			);

		chooser
			.addOption(
				"[comp] Amp Side Delay Strafe",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new WaitCommand(10),
					new ReadyShooter(Constants.Shooter.readyShootRear)
						.alongWith(Autonomous.choreo(Choreo.getTrajectory("AmpSideCenterNote.1"))),
					new ShootSpeaker(false).withTimeout(4),
					Autonomous.dynamic("AmpSideCenterNote.2"),

					Autonomous.choreo(Choreo.getTrajectory("AmpSideCenterNote.2")),
					new IntakeGround()
				)
			);

		chooser
			.addOption(
				"[comp] Two Note",
				new SequentialCommandGroup(
					new ShootSpeaker(false).withTimeout(4),
					new IntakeGround().withTimeout(2),
					new ShootSpeaker(false).withTimeout(4)
				)
			);

		return chooser;
	}

	public static Command setInitialPose(final String name) {
		final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(name);
		final Pose2d initial = traj.getPathPoses().get(0);

		return Commands.runOnce(() -> {
			Robot.cont.drivetrain
				.reset(
					new Pose2d(
						Autonomous.getPoseForAlliance(initial).getTranslation(),
						Robot.cont.drivetrain.est.getEstimatedPosition().getRotation()
					)
				);

			Logger.recordOutput("Drivetrain/Auto/x0", initial.getX());
			Logger.recordOutput("Drivetrain/Auto/y0", initial.getY());
			Logger.recordOutput("Drivetrain/Auto/r0", initial.getRotation().getDegrees());
		});
	}

	public static Command choreo(final ChoreoTrajectory trajectory) {
		final ChoreoControlFunction controller = Choreo
			.choreoSwerveController(
				Constants.Drivetrain.Auto.x.createController(), // PID to correct for field-relative X error
				Constants.Drivetrain.Auto.y.createController(), // PID to correct for field-relative Y error
				Constants.Drivetrain.Auto.theta.createController()
			);

		final Timer timer = new Timer();
		return new FunctionalCommand(timer::restart, () -> {
			final ChoreoTrajectoryState poseDemand = trajectory
				.sample(
					timer.get(),
					DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				);

			Logger.recordOutput("Choreo/DesiredPose", poseDemand.getPose());

			Robot.cont.drivetrain
				.controlRobotOriented(controller.apply(Robot.cont.drivetrain.blueOriginPose(), poseDemand));
		}, interrupted -> {
			timer.stop();
			if(interrupted) {
				Robot.cont.drivetrain.controlRobotOriented(new ChassisSpeeds());
			}
		}, () -> timer.hasElapsed(trajectory.getTotalTime()), Robot.cont.drivetrain);
	}

	public static Command path(final String name) {
		final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
		return AutoBuilder.followPath(choreoPath);
	}

	public static Command dynamic(final String next) {
		final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);

		return AutoBuilder
			.pathfindToPoseFlipped(
				traj.getPathPoses().get(0),
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					3,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			);
	}

	public static Command dynamicThen(final String next) {
		final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);

		return AutoBuilder
			.pathfindThenFollowPath(
				traj,
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					3,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			);
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
			return new Pose2d(
				initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
