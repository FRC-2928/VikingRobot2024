package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.superstructure.Superstructure;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.LimelightFXManager;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
	public final LoggedDashboardChooser<Command> autonomousChooser;
	public final LoggedDashboardChooser<String> driveModeChooser;
	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final DriveSubsystem drivetrain;
	public final Superstructure superstructure;
	public final Shooter shooter;
	public final Climber climber;

	public final LimelightFXManager fxm;

	public static boolean ledState = false;
	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard

		this.diag = new Diagnostics();
		this.drivetrain = new DriveSubsystem();
		this.superstructure = new Superstructure(this.drivetrain);
		this.shooter = new Shooter();
		this.climber = new Climber();
		this.fxm = new LimelightFXManager();

		this.diag.chirp(600, 500);
		this.diag.chirp(900, 500);

		this.drivetrain.configureJoystick(
			this.driverOI.driveAxial,
			this.driverOI.driveLateral,
			this.driverOI.driveFORX,
			this.driverOI.driveFORY,
			this::getDriveMode
		);

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			Autonomous.createAutonomousChooser()
		);
		this.driveModeChooser = new LoggedDashboardChooser<>(
			"Drive Mode",
			RobotContainer.createDriveModeChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();

		this.diag.configureControls();
	}

	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }

	public String getDriveMode() { return this.driveModeChooser.get(); }

	public static SendableChooser<String> createDriveModeChooser() {
		final SendableChooser<String> chooser = new SendableChooser<>();
		chooser.addOption("Swerve Drive", "Swerve Drive");
		chooser.addOption("Field Oriented", "Field Oriented");
		chooser.setDefaultOption("Swerve Drive", "Swerve Drive");
		return chooser;
	}
}
