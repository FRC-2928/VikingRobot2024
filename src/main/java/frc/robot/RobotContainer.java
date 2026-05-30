package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autonomous.AutoOption;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.superstructure.Superstructure;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightFXManager;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
	public final LoggedDashboardChooser<AutoOption> autonomousChooser;
	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final CommandSwerveDrivetrain drivetrain;
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
		this.drivetrain = new CommandSwerveDrivetrain();
		this.shooter = new Shooter();
		this.superstructure = new Superstructure(this.drivetrain, this.shooter);
		this.climber = new Climber();
		this.fxm = new LimelightFXManager();

		this.diag.chirp(600, 500);
		this.diag.chirp(900, 500);

		this.drivetrain.configureJoystick(
			this.driverOI.driveAxial,
			this.driverOI.driveLateral,
			this.driverOI.driveRotation
		);

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			Autonomous.createAutonomousChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();

		this.diag.configureControls();
	}

	public Command getAutonomousCommand() {
		AutoOption opt = this.autonomousChooser.get();
		return opt != null ? opt.command() : null;
	}

	public Optional<Pose2d> getAutoStartPose() {
		AutoOption opt = this.autonomousChooser.get();
		if (opt == null) return Optional.empty();
		Pose2d p = opt.startPose();
		// A zero/origin pose means no defined start — don't reset
		return p.getTranslation().getNorm() > 0.01 ? Optional.of(p) : Optional.empty();
	}
}
