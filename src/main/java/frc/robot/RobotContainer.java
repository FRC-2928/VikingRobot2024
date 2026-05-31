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
	private static RobotContainer sInstance = null;

	public static synchronized RobotContainer getInstance() {
		if (sInstance == null) {
			sInstance = new RobotContainer();
		}
		return sInstance;
	}

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
	private RobotContainer() {

		Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard

		this.diag = new Diagnostics();

		// Superstructure MUST be constructed before subsystems. WPILib's CommandScheduler
		// calls periodic() in registration order, and Superstructure must run first to
		// batch-refresh CAN signals and resolve goals before subsystems read their inputs.
		//
		// Future: Superstructure should own subsystem construction internally and drive
		// their periodic() calls explicitly, removing the scheduler ordering dependency.
		// That refactor is blocked on eliminating Robot.cont (task 10).
		this.superstructure = new Superstructure();
		this.drivetrain = new CommandSwerveDrivetrain();
		this.shooter = new Shooter();
		this.climber = new Climber();
		this.superstructure.setSubsystems(this.drivetrain, this.shooter, this.climber);
		this.fxm = new LimelightFXManager();

		this.superstructure.registerSignals(this.shooter, this.shooter.getStatusSignals());
		this.superstructure.registerSignals(this.climber, this.climber.getStatusSignals());

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
