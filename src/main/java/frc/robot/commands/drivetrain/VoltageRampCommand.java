package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class VoltageRampCommand extends Command {
	public VoltageRampCommand() {
		this.addRequirements(Robot.cont.drivetrain);
		VoltageRampCommand.voltage = 0;
	}

	private static double voltage;

	@Override
	public void initialize() { VoltageRampCommand.voltage = 0; }

	@Override
	public void execute() {
		// runCharacterization is not available in DriveSubsystem; use CTRE SysId routines instead
		VoltageRampCommand.voltage += 0.005;
	}

	@Override
	public boolean isFinished() { return VoltageRampCommand.voltage > 0.5; }

	@Override
	public void end(final boolean interrupted) { Robot.cont.drivetrain.halt(); }

}
