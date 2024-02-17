package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOPigeon2;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.ModuleIO;
import frc.robot.subsystems.ModuleIOSim;
import frc.robot.subsystems.ModuleIOTalonFX;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

	public final LoggedDashboardChooser<Command> autonomousChooser; 

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	// public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Drivetrain drivetrain;
    public final Shooter shooter = new Shooter();

	public RobotContainer() { 
		
		switch (Constants.currentMode) {
			case REAL:
				// Real robot, instantiate hardware IO implementations      
				drivetrain = new Drivetrain(
					new GyroIOPigeon2(),
					new ModuleIOTalonFX(SwerveModule.Place.FrontLeft),
					new ModuleIOTalonFX(SwerveModule.Place.FrontRight),
					new ModuleIOTalonFX(SwerveModule.Place.BackLeft),
					new ModuleIOTalonFX(SwerveModule.Place.BackRight));
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drivetrain =
					new Drivetrain(
						null,
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim());
				break;

			default:
				// Replayed robot, disable IO implementations
				drivetrain =
					new Drivetrain(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {});
				break;
		}

		this.autonomousChooser = new LoggedDashboardChooser<>("Autonomous Routine", AutonomousRoutines.createAutonomousChooser(this.drivetrain));
	}

	public void configureDriverControls() {
		// this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.gyro.setYaw(0)));
		this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.resetGyro())); // Y Button
		this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI)); // Left Bumper
        this.driverOI.shoot.whileTrue(new ConditionalCommand(new Intake(), new Shoot(), this.shooter.launcher.getSensorCollection()::isFwdLimitSwitchClosed)); // Left Bumper
	}
	
	public void teleop() { this.drivetrain.setDefaultCommand(new JoystickDrive(this.drivetrain, this.driverOI)); }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return this.autonomousChooser.get();
	}

}
