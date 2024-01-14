package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	private Command autonomousCommand;
	private final LoggedDashboardChooser<Command> autoChooser;

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Drivetrain drivetrain = new Drivetrain();

	public RobotContainer() {
		this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI));
		this.driverOI.resetFOD.whileTrue(new RunCommand(() -> {
			this.drivetrain.gyro.setYaw(0);
			System.out.printf("rst %s\n", this.drivetrain.gyro.getRotation2d());
		}));

		this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", null);
	}

	// public void init() {}

	// public void disabled() {}

	// public void enabled() {}

	public void auto() {
		this.autonomousCommand = null;

		if(this.autonomousCommand != null) this.autonomousCommand.schedule();
	}

	public void teleop() {
		this.drivetrain.setDefaultCommand(new Drive(this.drivetrain, this.driverOI));
	}

	public void test() { CommandScheduler.getInstance().cancelAll(); }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

}
