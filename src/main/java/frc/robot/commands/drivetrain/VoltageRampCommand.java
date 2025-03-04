package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class VoltageRampCommand extends Command {
	public VoltageRampCommand() {
		this.addRequirements(RobotContainer.getInstance().drivetrain);
		VoltageRampCommand.voltage = 0;
	}

	private static double voltage;

	@Override
	public void initialize() { VoltageRampCommand.voltage = 0; }

	@Override
	public void execute() {
		RobotContainer.getInstance().drivetrain.runCharacterization(VoltageRampCommand.voltage);
		VoltageRampCommand.voltage += 0.005;
	}

	@Override
	public boolean isFinished() { return VoltageRampCommand.voltage > 0.5; }

	@Override
	public void end(final boolean interrupted) { RobotContainer.getInstance().drivetrain.halt(); }

}
