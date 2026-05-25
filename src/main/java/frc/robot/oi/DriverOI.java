package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.commands.drivetrain.TestDrive;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.shooter.ShooterGoal;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		}
		this.manualRotation = this.controller.rightStick();

		this.shootSpeaker = this.controller.leftTrigger();
		this.shootAmp = this.controller.leftBumper();
		this.intake = this.controller.rightTrigger();

		this.ferry = this.controller.rightBumper();

		this.resetFOD = this.controller.y();

		this.lockWheels = this.controller.x();
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;
	public final Trigger manualRotation;

	public final Trigger shootSpeaker;
	public final Trigger shootAmp;
	public final Trigger intake;

	public final Trigger lockWheels;

	public final Trigger resetFOD;

	public final Trigger ferry;

	public void configureControls() {
		this.shootSpeaker.whileTrue(
			Robot.cont.superstructure.setShooterIntentCommand(ShooterGoal.SHOOT_SPEAKER)
				.alongWith(Robot.cont.superstructure.setDriveIntentCommand(DriveGoal.AIM_SPEAKER)));
		this.shootAmp.whileTrue(Robot.cont.superstructure.setShooterIntentCommand(ShooterGoal.AMP));
		this.intake.whileTrue(Robot.cont.superstructure.setShooterIntentCommand(ShooterGoal.INTAKE));

		this.lockWheels
			.onTrue(new InstantCommand(() -> {
				RobotContainer.ledState = true;
				this.hid.setRumble(RumbleType.kBothRumble, 0.25);
			}))
			.onFalse(new InstantCommand(() -> {
				RobotContainer.ledState = false;
				this.hid.setRumble(RumbleType.kBothRumble, 0);
			}))
			.whileTrue(Robot.cont.superstructure.setDriveIntentCommand(DriveGoal.LOCK));
		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));

		this.ferry.whileTrue(Robot.cont.superstructure.setShooterIntentCommand(ShooterGoal.FERRY));

		this.controller.a().whileTrue(new TestDrive());
	}
}
