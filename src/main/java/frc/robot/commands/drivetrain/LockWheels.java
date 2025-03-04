package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class LockWheels extends Command {
	private final Drivetrain drivetrain = RobotContainer.getInstance().drivetrain;
	private final DriverOI oi = RobotContainer.getInstance().driverOI;

	public LockWheels() { this.addRequirements(this.drivetrain); }

	@Override
	public void execute() {
		this.drivetrain.control(Drivetrain.State.locked());
		if(this.oi != null) this.oi.hid.setRumble(RumbleType.kBothRumble, 0.25);
	}

	@Override
	public void end(final boolean interrupted) {
		this.drivetrain.control(Drivetrain.State.forward());
		if(this.oi != null) this.oi.hid.setRumble(RumbleType.kBothRumble, 0);
	}
}
