package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
public class TestDrive extends Command {
	private final RobotContainer rc = RobotContainer.getInstance();

	public TestDrive() { this.addRequirements(rc.drivetrain); }

	@Override
	public void execute() { rc.drivetrain.driveFieldOriented(new ChassisSpeeds(2, 0, 0)); }

	@Override
	public void end(final boolean interrupted) { rc.drivetrain.halt(); }
}
