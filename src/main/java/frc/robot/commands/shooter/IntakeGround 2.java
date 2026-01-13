package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.oi.BaseOI;
import frc.robot.subsystems.ShooterIO.Demand;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends Command {
	public static double lastTime = 0; // this is a bad way to do this but its necessary for right now, please do real path planning in the future

	public IntakeGround(final boolean correction) {
		this.haptics.type = RumbleType.kBothRumble;
		this.haptics.interval = 1;
		this.haptics.dutyCycle = 1;
		this.haptics.powerTrue = 1;
		this.haptics.powerFalse = 0;

		this.correction = correction;

		this.addRequirements(Robot.cont.shooter);
		if(correction) this.addRequirements(Robot.cont.drivetrain);
	}

	private final BaseOI.Haptics haptics = new BaseOI.Haptics(Robot.cont.driverOI.hid);

	public final boolean correction;

	@Override
	public void execute() {
		final boolean pivotReady = Math
			.abs(
				Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.intakeGround.in(Units.Degrees)
			) <= 1.5;

		Robot.cont.shooter.io.rotate(Constants.Shooter.intakeGround);
		Robot.cont.shooter.io.runFlywheels(-0.25);
		Robot.cont.shooter.io.runFeeder(Demand.Reverse);
		Robot.cont.shooter.io.runIntake(pivotReady ? Demand.Forward : Demand.Halt);

		if(this.correction)
			Robot.cont.drivetrain
				.control(
					Robot.cont.drivetrain.joystickSpeeds
						.plus(
							Robot.cont.drivetrain
								.rod(
									new ChassisSpeeds(
										this.calculateSpeedX(),
										Robot.cont.drivetrain.limelightNote
											.getTargetHorizontalOffset()
											.in(Units.Rotations)
											* 10,
										0
									).times(pivotReady ? 1 : 1)
								)
						)
				);

		this.haptics.update();
		
	}
	public double calculateSpeedX(){
		Logger.recordOutput("Drivetrain/auto/SpeedXIntakeGroun",(-10/(Math.abs(Robot.cont.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1)));
		return( 
				(-10/(Math.abs(Robot.cont.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1))
		);
	}
	
	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
		Robot.cont.shooter.io.runIntake(Demand.Halt);

		Robot.cont.drivetrain.control(new ChassisSpeeds());

		this.haptics.stop();
	}

	@Override
	public boolean isFinished() { return Robot.cont.shooter.inputs.holdingNote; }
}
