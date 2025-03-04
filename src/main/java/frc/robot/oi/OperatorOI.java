package frc.robot.oi;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePieceType;
import frc.robot.commands.drivetrain.JoystickDrive;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climbMotion = this.controller::getLeftY;

		this.climberOverrideLower = this.controller.povDown();
		this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();
		
		this.intakeIn = this.controller.a();
		this.intakeOut = this.controller.b();
		this.slowedTrigger = this.controller.x();
		this.halt = this.controller.y();
		this.foc = this.controller.rightBumper();
		this.resetAngle = this.controller.back();
		this.toggleClimb = this.controller.leftBumper();
		this.climbModeOn = new Trigger(() -> (RobotContainer.getInstance().elevator.hasCurrentGamePieceType(GamePieceType.CAGE)));
		this.toggleReefHeightDown = this.controller.povDown();
		this.toggleReefHeightUp = this.controller.povUp();
		this.passOffCoral = this.controller.rightTrigger();
		this.alignElevatorCoral = new Trigger(() -> (this.controller.povRight().getAsBoolean())).and(RobotContainer.getInstance().driverOI.holdingCoral);
		this.alignElevatorAlgaeL2 = new Trigger(() -> (this.controller.povRight().getAsBoolean())).and(RobotContainer.getInstance().driverOI.holdingCoral);
		this.alignElevatorAlgaeL3 = new Trigger(() -> (this.controller.povLeft().getAsBoolean())).and(RobotContainer.getInstance().driverOI.holdingCoral);
	}

	public final Trigger climbModeOn;
	public final Trigger foc;
	public final Trigger climberOverrideLower;
	public final Trigger climberOverrideRaise;
	public final Trigger resetAngle;
	public final Trigger initializeClimber;
	public final Trigger toggleClimb;
	// public final Trigger raiseElevator;
	// public final Trigger lowerElevator;
	public final Trigger intakeOut;
	public final Trigger intakeIn;
	public final DoubleSupplier climbMotion;
	public final Trigger toggleReefHeightUp;
	public final Trigger toggleReefHeightDown;
	public final Trigger passOffCoral;
	public final Trigger alignElevatorCoral;
	public final Trigger alignElevatorAlgaeL2;
	public final Trigger alignElevatorAlgaeL3;
	public final Trigger halt;
	public final Trigger slowedTrigger;

	public void configureControls() {
		this.slowedTrigger.whileTrue(new JoystickDrive(RobotContainer.getInstance().drivetrain, 0.3));
		this.halt.whileTrue(new RunCommand(() -> RobotContainer.getInstance().drivetrain.halt()));
		toggleClimb.onTrue(RobotContainer.getInstance().elevator.toggleClimbMode());
		this.climbModeOn.whileTrue(RobotContainer.getInstance().elevator.doClimb(this.climbMotion));
		this.toggleReefHeightDown.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightDown));
		this.toggleReefHeightUp.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightUp));
		this.passOffCoral.whileTrue(RobotContainer.getInstance().passCoral());
		this.alignElevatorCoral.whileTrue(
			RobotContainer.getInstance().telePositionForCoralOveride()
		);
		this.alignElevatorAlgaeL2.whileTrue(
			RobotContainer.getInstance().telePositionForAlgaeOverideL2()
		);
		this.alignElevatorAlgaeL3.whileTrue(
			RobotContainer.getInstance().telePositionForAlgaeOverideL3()
		);
	}
}
//change elevator height
//create dashboard w/ shuffle board