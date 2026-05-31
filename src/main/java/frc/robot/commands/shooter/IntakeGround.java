package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;import frc.robot.oi.BaseOI;
import frc.robot.subsystems.ShooterIO.Demand;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends Command {
	public static double lastTime = 0; // this is a bad way to do this but its necessary for right now, please do real path planning in the future
	private final RobotContainer rc = RobotContainer.getInstance();

	public IntakeGround(final boolean correction) {
		this.haptics.type = RumbleType.kBothRumble;
		this.haptics.interval = 1;
		this.haptics.dutyCycle = 1;
		this.haptics.powerTrue = 1;
		this.haptics.powerFalse = 0;

		this.correction = correction;

		this.addRequirements(rc.shooter);
		if(correction) this.addRequirements(rc.drivetrain);
	}

	private final BaseOI.Haptics haptics = new BaseOI.Haptics(rc.driverOI.hid);

	public final boolean correction;

	@Override
	public void execute() {
		final boolean pivotReady = Math
			.abs(
				rc.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.intakeGround.in(Units.Degrees)
			) <= 1.5;

		rc.shooter.io.rotate(Constants.Shooter.intakeGround);
		rc.shooter.io.runFlywheels(-0.35);
		rc.shooter.io.runFeeder(Demand.Reverse);
		rc.shooter.io.runIntake(pivotReady ? Demand.Forward : Demand.Halt);

		if(this.correction)
			rc.drivetrain.driveFieldOriented(
				rc.drivetrain.joystickSpeeds
					.plus(
						rc.drivetrain.robotToField(
							new ChassisSpeeds(
								this.calculateSpeedX(),
								rc.drivetrain.limelightNote
									.getTargetHorizontalOffset()
									.in(Units.Rotations)
									* 10,
								0
							)
						)
					)
			);

		this.haptics.update();
		
	}
	public double calculateSpeedX(){
		Logger.recordOutput("Drivetrain/auto/SpeedXIntakeGroun",(-10/(Math.abs(rc.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1)));
		return( 
				(-10/(Math.abs(rc.drivetrain.limelightNote.getTargetHorizontalOffset().in(Units.Degrees))+1))
		);
	}
	
	@Override
	public void end(final boolean interrupted) {
		rc.shooter.io
			.rotate(
				rc.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		rc.shooter.io.runFlywheels(0);
		rc.shooter.io.runFeeder(Demand.Halt);
		rc.shooter.io.runIntake(Demand.Halt);

		rc.drivetrain.driveFieldOriented(new ChassisSpeeds());

		this.haptics.stop();
	}

	@Override
	public boolean isFinished() { return rc.shooter.inputs.holdingNote; }
}
