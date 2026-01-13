package frc.robot;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

//import choreo.*;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.commands.shooter.*;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"[comp] Five Note",
				new SequentialCommandGroup(
					new ReadyShooter(Units.Degrees.of(114), true),
					Autonomous.setInitialPose("MiddleFiveNote.1"),
					new WaitCommand(0.2),
					new ShootFixed(false, 2),
					Autonomous
						.path("MiddleFiveNote.1")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyIntake, false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.3")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.5")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("MiddleFiveNote.5"),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("MiddleFiveNote.6")
						.deadlineWith(
							new WaitCommand(2).andThen(new ReadyShooter(Constants.Shooter.readyShootRear, true))
						),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] Source Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("SourceSideCenterNote.1"),
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootFixed(false, 2),
					Autonomous.path("SourceSideCenterNote.1"),
					new IntakeGround(true).withTimeout(2),
					Autonomous.dynamic("SourceSideCenterNote.2"),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear, true),
						Autonomous.path("SourceSideCenterNote.2")
					),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] Drive",
				new SequentialCommandGroup(new ReadyShooter(Constants.Shooter.readyShootRear, true), new DriveTime(1))
			);

		chooser
			.addOption(
				"[comp] Shoot/Drive",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2),
					new DriveTime(1)
				)
			);

		chooser
			.addOption(
				"[comp] Shoot",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Squirrel Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					new WaitCommand(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Delayed Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new WaitCommand(10),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] The Jamp",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("jamp.1"),
					new ShootFixedDiag(false, 2),
					Autonomous.path("jamp.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("jamp.2")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("jamp.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("jamp.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] The Close Jamp",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("closeJamp.1"),
					new ShootFixedDiag(false, 2),
					Autonomous.path("closeJamp.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("closeJamp.2")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootFront, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("closeJamp.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("closeJamp.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootFront, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] The Jource",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("jource.1"),
					new ShootFixedDiag(false, 2),
					Autonomous.path("jource.1").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous.dynamicThen("jource.2"),
					new ShootSpeaker(false, 2),
					Autonomous.path("jource.3").deadlineWith(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3)
				)
			);

		chooser
			.addOption(
				"[comp] Two Note",
				new SequentialCommandGroup(
					new ShootFixed(false, 2),
					new IntakeGround(true).withTimeout(1.5),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Source Side Bulldoze",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("Bulldoze.1"),
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootFixed(false, 2),
					Autonomous.path("Bulldoze.1"),
					new IntakeGround(true).withTimeout(2)
				)
			);
		chooser.addOption("[testing] Intake Only", new SequentialCommandGroup(new IntakeGround(true).withTimeout(2)));
		chooser
			.addOption(
				"[testing] forward only",
				new SequentialCommandGroup(Autonomous.setInitialPose("forwardBack.1"), Autonomous.path("forwardBack.1"))
			);
		chooser
			.addOption(
				"[testing] forward Back",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("forwardBack.1"),
					Autonomous.path("forwardBack.1"),
					new LookForNote(Units.Radians.of(Math.PI/3)),
					new IntakeGround(true).withTimeout(2),
					Autonomous.dynamic("forwardBack.2"),
					Autonomous.path("forwardBack.2")
				)
			);
		chooser.addOption(
			"[testing] LookForNote", 
			new SequentialCommandGroup(
				new LookForNote(Units.Radians.of(Math.PI/4)),
				(Robot.cont.drivetrain.limelightNote.hasValidTargets() ?  new IntakeGround(true).withTimeout(4) : new LookForNote(Units.Radians.of(-Math.PI/2)))
			)
		);
		chooser
			.addOption("[testing] dynamic path back", new SequentialCommandGroup(Autonomous.dynamic("forwardBack.2")));
		chooser.addOption("[testing] return to start", new SequentialCommandGroup(Autonomous.path("forwardBack.2")));
		chooser
			.addOption(
				"[testing] forward path too back",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("forwardBack.1"),
					Autonomous.path("forwardBack.1"),
					new IntakeGround(true).withTimeout(2),
					Autonomous.dynamic("forwardBack.2")
				)
			);
		chooser.addOption("[testing] voltage ramp", new VoltageRampCommand());
		return chooser;
	}

	public static Pose2d getTrajStartPose2d(Optional<? extends Trajectory<?>> traj) {
		Pose2d startingPose = new Pose2d();
		if (traj.isPresent()) {
			startingPose = traj.get().getInitialPose(true).orElse(new Pose2d());
		}
		return startingPose;
	}

	public static Command setInitialPose(final String name) {
		//final Trajectory traj = Choreo.loadTrajectory(name);
		final Pose2d initial = getTrajStartPose2d(Choreo.loadTrajectory(name));

		return Commands.runOnce(() -> {
			Robot.cont.drivetrain.reset(Autonomous.getPoseForAlliance(initial));

			Logger.recordOutput("Drivetrain/Auto/x0", initial.getX());
			Logger.recordOutput("Drivetrain/Auto/y0", initial.getY());
			Logger.recordOutput("Drivetrain/Auto/r0", initial.getRotation().getDegrees());
		});
	}

	public static Command path(final String name) {
		try {
			final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
		return AutoBuilder.followPath(choreoPath);
		}
		catch(IOException | ParseException e) {
			// In case path can't be read, throw runtime exception
			throw new RuntimeException();
		}
	}

	public static Command dynamic(final String next, final double maxvel) {
		Optional<? extends Trajectory<?>> trajectory = Choreo.loadTrajectory(next);
		Pose2d startingPose = new Pose2d();
		if (trajectory.isPresent()) {
			startingPose = trajectory.get().getInitialPose(true).orElse(new Pose2d());
		}
		final Pose2d finalStartPose = startingPose;
		return AutoBuilder
			.pathfindToPoseFlipped(
				startingPose,
				new PathConstraints(maxvel, 2, Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond), 2)
			)
			.alongWith(
				new InstantCommand(() -> Logger.recordOutput("Drivetrain/Auto/DynamicTarget", finalStartPose))
			);
	}

	public static Command dynamic(final String next) {
		return Autonomous.dynamic(next, Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond));
	}

	public static Command dynamicThen(final String next) {
		try{
			final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);
		Optional<? extends Trajectory<?>> trajectory = Choreo.loadTrajectory(next);
		Pose2d startingPose = new Pose2d();
		if (trajectory.isPresent()) {
			startingPose = trajectory.get().getInitialPose(true).orElse(new Pose2d());
		}
		final Pose2d finalStartPose = startingPose;
		return AutoBuilder
			.pathfindThenFollowPath(
				traj,
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					3,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			)
			.alongWith(
				new InstantCommand(
					() -> Logger
						.recordOutput("Drivetrain/Auto/DynamicTarget", finalStartPose)
				)
			);
		}
		catch(IOException | ParseException e) {
			// In case path can't be read, throw runtime exception
			throw new RuntimeException();
		}
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
