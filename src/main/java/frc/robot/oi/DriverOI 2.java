package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.Constants.Mode;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.drivetrain.TestDrive;
import frc.robot.commands.shooter.IntakeGround;
import frc.robot.commands.shooter.PrepareAmpShot;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.FinishAmpShot;
import frc.robot.commands.shooter.ShootFixed;
import frc.robot.commands.shooter.ShootSpeaker;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = () -> this.hid.getRawAxis(2);
			this.driveFORY = () -> this.hid.getRawAxis(3);
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
		this.shootSpeaker.whileTrue(new ShootSpeaker(true));
		this.shootAmp
			.onTrue(new PrepareAmpShot().withTimeout(0.3))
			.whileTrue(new ShootAmp())
			.onFalse(new FinishAmpShot().withTimeout(0.6));
		this.intake.whileTrue(new IntakeGround(true));

		this.lockWheels.onTrue(new InstantCommand(() -> Robot.cont.ledState = true))
		.onFalse(new InstantCommand(() -> Robot.cont.ledState = false));
	//	this.lockWheels.whileTrue(new LockWheels());
		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));

		this.ferry.whileTrue(new ShootFixed(() -> Units.Degrees.of(Tuning.ferryAngle.get()), false, 0));

		this.controller.a().whileTrue(new TestDrive());
	}
}
