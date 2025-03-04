package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.oi.BaseOI;
import frc.robot.subsystems.Intake;
import org.littletonrobotics.junction.Logger;

// THIS CLASS IS ONLY FOR REFERENCE FROM 2024. DO NOT USE IT
public class IntakeGround extends Command {
	public static double lastTime = 0; // this is a bad way to do this but its necessary for right now, please do real path planning in the future

	public IntakeGround(final boolean correction) {
		this.haptics.type = RumbleType.kBothRumble;
		this.haptics.interval = 1;
		this.haptics.dutyCycle = 1;
		this.haptics.powerTrue = 1;
		this.haptics.powerFalse = 0;

		this.correction = correction;

		this.addRequirements(RobotContainer.getInstance().intake);
		if(correction) this.addRequirements(RobotContainer.getInstance().drivetrain);
	}

	private final BaseOI.Haptics haptics = new BaseOI.Haptics(RobotContainer.getInstance().driverOI.hid);

	public final boolean correction;

	@Override
	public void execute() {
		// final boolean pivotReady = Math
		// 	.abs(
		// 		RobotContainer.getInstance().intake.inputs.angle.in(Units.Degrees) - Constants.Intake.max.in(Units.Degrees)
		// 	) <= 1.5;

		// RobotContainer.getInstance().intake.rotate(Constants.Intake.max);
		// RobotContainer.getInstance().shooter.io.runFlywheels(-0.25);
		// RobotContainer.getInstance().shooter.io.runFeeder(Demand.Reverse);
		// RobotContainer.getInstance().shooter.io.runIntake(pivotReady ? Demand.Forward : Demand.Halt);

		// if(this.correction)
		// 	RobotContainer.getInstance().drivetrain
		// 		.control(
		// 			RobotContainer.getInstance().drivetrain.joystickSpeeds
		// 				.plus(
		// 					RobotContainer.getInstance().drivetrain
		// 						.rod(
		// 							new ChassisSpeeds(
		// 								this.calculateSpeedX(),
		// 								RobotContainer.getInstance().drivetrain.limelightNote
		// 									.getTargetHorizontalOffset()
		// 									.in(Units.Rotations)
		// 									* 10,
		// 								0
		// 							).times(pivotReady ? 1 : 1)
		// 						)
		// 				)
		// 		);

		// this.haptics.update();
		
	}
	public double calculateSpeedX() {
		// Logger.recordOutput("Drivetrain/auto/SpeedXIntakeGroun",(-10/(Math.abs(RobotContainer.getInstance().drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1)));
		// return( 
		// 		(-10/(Math.abs(RobotContainer.getInstance().drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1))
		// );
		return 0d;
	}
	
	@Override
	public void end(final boolean interrupted) {
		// RobotContainer.getInstance().shooter.io
		// 	.rotate(
		// 		RobotContainer.getInstance().shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
		// 	);
		// RobotContainer.getInstance().shooter.io.runFlywheels(0);
		// RobotContainer.getInstance().shooter.io.runFeeder(Demand.Halt);
		// RobotContainer.getInstance().shooter.io.runIntake(Demand.Halt);

		// RobotContainer.getInstance().drivetrain.control(new ChassisSpeeds());

		// this.haptics.stop();
	}

	@Override
	public boolean isFinished() { return false;/*return RobotContainer.getInstance().shooter.inputs.holdingNote;*/ }	
}

