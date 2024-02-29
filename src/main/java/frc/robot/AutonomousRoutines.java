package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.shooter.IntakeGround;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public final class AutonomousRoutines {
	public static SendableChooser<Command> createAutonomousChooser() {
		final Drivetrain drivetrain = Robot.cont.drivetrain;
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"Drive test trajectory",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("test")),
					AutonomousRoutines.choreo(Choreo.getTrajectory("test"))
				)
			);

		chooser
			.addOption(
				"Four Note Close",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("fourNoteClose.1")),
					new ShootSpeaker(),
					AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.1")),
					new IntakeGround(true),
					new ParallelCommandGroup(
						AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.2")),
						new ShootSpeaker()
					),
					new IntakeGround(true),
					new ShootSpeaker(),
					AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.3")),
					new IntakeGround(true),
					new ShootSpeaker()
				)
			);

		chooser
			.addOption(
				"Two Note Center D",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("TwoNoteCenterD.1")),
					new ShootSpeaker(),
					AutonomousRoutines.choreo(Choreo.getTrajectory("TwoNoteCenterD.1")),
					new IntakeGround(true),
					AutonomousRoutines.choreo(Choreo.getTrajectory("TwoNoteCenterD.2")),
					new ShootSpeaker()
				)
			);

		return chooser;
	}

	public static Command setInitialPose(final ChoreoTrajectory trajectory) {
		return Commands.runOnce(() -> {
			Robot.cont.drivetrain
				.reset(
					new Pose2d(
						AutonomousRoutines.getPoseForAlliance(trajectory.getInitialPose()).getTranslation(),
						Robot.cont.drivetrain.est.getEstimatedPosition().getRotation()
					)
				);

			Logger.recordOutput("Drivetrain/Choreo/x0", trajectory.getInitialPose().getX());
			Logger.recordOutput("Drivetrain/Choreo/y0", trajectory.getInitialPose().getY());
			Logger.recordOutput("Drivetrain/Choreo/r0", trajectory.getInitialPose().getRotation().getDegrees());
		});
	}

	public static Command choreo(final ChoreoTrajectory trajectory) {
		return Choreo
			.choreoSwerveCommand(
				trajectory,
				Robot.cont.drivetrain::blueOriginPose, // A function that returns the current field-relative pose of the robot:
				Constants.Drivetrain.Choreo.x.createController(), // PID to correct for field-relative X error
				Constants.Drivetrain.Choreo.y.createController(), // PID to correct for field-relative Y error
				Constants.Drivetrain.Choreo.theta.createController(), // PID to correct for rotation error
				Robot.cont.drivetrain::control,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
				Robot.cont.drivetrain // The subsystem(s) to require, typically your drive subsystem only
			);
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return new Pose2d(
				initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
