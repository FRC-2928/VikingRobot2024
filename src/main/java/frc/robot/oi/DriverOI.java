package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.TestDrive;
import frc.robot.subsystems.climber.ClimberGoal;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.superstructure.GoalResolver;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;
		this.driveRotation = this.controller::getRightX;

		this.shootSpeaker = this.controller.leftTrigger();
		this.shootAmp = this.controller.leftBumper();
		this.intake = this.controller.rightTrigger();

		this.ferry = this.controller.rightBumper();

		this.resetHeading = this.controller.y();

		this.lockWheels = this.controller.x();

	this.climberExtend = this.controller.povUp();
	this.climberRetract = this.controller.povDown();
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;
	public final Supplier<Double> driveRotation;

	public final Trigger shootSpeaker;
	public final Trigger shootAmp;
	public final Trigger intake;

	public final Trigger lockWheels;

	public final Trigger resetHeading;

	public final Trigger ferry;

	public final Trigger climberExtend;
	public final Trigger climberRetract;

	public void configureControls() {
		final GoalResolver resolver = RobotContainer.getInstance().superstructure.resolver;

		this.shootSpeaker
			.onTrue(new InstantCommand(() -> {
				resolver.setShooterIntent(ShooterGoal.SHOOT_SPEAKER);
				resolver.setDriveIntent(DriveGoal.AIM_SPEAKER);
			}))
			.onFalse(new InstantCommand(() -> {
				resolver.setShooterIntent(ShooterGoal.HOME);
				resolver.setDriveIntent(DriveGoal.TELEOP);
			}));

		this.shootAmp
			.onTrue(new InstantCommand(() -> resolver.setShooterIntent(ShooterGoal.AMP)))
			.onFalse(new InstantCommand(() -> resolver.setShooterIntent(ShooterGoal.HOME)));

		this.intake
			.onTrue(new InstantCommand(() -> {
				resolver.setShooterIntent(ShooterGoal.INTAKE);
				resolver.setDriveIntent(DriveGoal.TRACK_NOTE);
			}))
			.onFalse(new InstantCommand(() -> {
				resolver.setShooterIntent(ShooterGoal.HOME);
				resolver.setDriveIntent(DriveGoal.TELEOP);
			}));

		this.lockWheels
			.onTrue(new InstantCommand(() -> {
				resolver.setDriveIntent(DriveGoal.LOCK);
				RobotContainer.ledState = true;
				this.hid.setRumble(RumbleType.kBothRumble, 0.25);
			}))
			.onFalse(new InstantCommand(() -> {
				resolver.setDriveIntent(DriveGoal.TELEOP);
				RobotContainer.ledState = false;
				this.hid.setRumble(RumbleType.kBothRumble, 0);
			}));

		this.resetHeading.onTrue(new InstantCommand(RobotContainer.getInstance().drivetrain::resetAngle));

		this.ferry
			.onTrue(new InstantCommand(() -> resolver.setShooterIntent(ShooterGoal.FERRY)))
			.onFalse(new InstantCommand(() -> resolver.setShooterIntent(ShooterGoal.HOME)));

		this.controller.a().whileTrue(new TestDrive());

		this.climberExtend
			.onTrue(new InstantCommand(() -> resolver.setClimberIntent(ClimberGoal.DEPLOY)))
			.onFalse(new InstantCommand(() -> resolver.setClimberIntent(ClimberGoal.IDLE)));

		this.climberRetract
			.onTrue(new InstantCommand(() -> resolver.setClimberIntent(ClimberGoal.RETRACT)))
			.onFalse(new InstantCommand(() -> resolver.setClimberIntent(ClimberGoal.IDLE)));
	}
}
