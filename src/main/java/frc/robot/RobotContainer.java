package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoChooser;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AlgaePosition;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.BananaFlywheels;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	public LoggedDashboardChooser<String> driveModeChooser;
	public DriverOI driverOI;
	public OperatorOI operatorOI;

	public Diagnostics diag;
	public Drivetrain drivetrain;
	public Elevator elevator;
	public Intake intake;
	public BananaFlywheels bananaFlywheels;
	
	public AutoChooser autoChooser;
	private static RobotContainer sInstance = null;

	public static synchronized RobotContainer getInstance() {
		if (sInstance != null) {
			return sInstance;
		}

		sInstance = new RobotContainer();
		sInstance.init();
		return sInstance;
	}

	private RobotContainer() {
		// Empty
	}

	private void init() {
		this.driverOI = new DriverOI(new CommandXboxController(0));
		this.operatorOI = new OperatorOI(new CommandXboxController(1));
		Tuning.algaePivotHome.get();
		this.diag = new Diagnostics();
		Tuning.intakeSpeed.get(); // load the class to put the tuning controls on the dashboard
		this.drivetrain = new Drivetrain();
		this.elevator = new Elevator();
		this.intake = new Intake();
		this.bananaFlywheels = new BananaFlywheels();

		autoChooser = Autonomous.getChoreoAutoChooser();
		SmartDashboard.putData("Autonomous Routine", autoChooser);
		RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

		this.driveModeChooser = new LoggedDashboardChooser<>(
			"Drive Mode",
			JoystickDrive.createDriveModeChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();
	}

	public Command autoScoreCoral(ReefPosition reefPos) {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				// Center Limelight
				CenterLimelight.centerLimeLightPosition(reefPos),
				// Set elevator
				this.elevator.goToReefHeight(GamePieceType.CORAL)
			),
			Commands.deadline(
				// Scores coral
				this.bananaFlywheels.scoreHeldCoral(),
				// Holds elevator in place
				this.elevator.goToGamePieceHeight(GamePieceType.CORAL)
			),
			new InstantCommand(() -> this.elevator.onEjectCoral(), this.elevator)
		);
	}

	public Command telePositionForCoralLeft() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.CORAL),
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightLeft(),
				drivetrain.slowMode()
			)
		);
	}
	public Command telePositionForCoralOveride() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.CORAL),
			drivetrain.slowMode()
		);
	}

	public Command telePositionForCoralRight() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.CORAL),
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightRight(),
				drivetrain.slowMode()
			)
		);
	}

	public Command telePositionForAlgae() {
		return new SequentialCommandGroup(
			this.elevator.setTargetAlgaeLevelCommand(RobotContainer.getInstance().drivetrain.getAlgaeHeight()),
			CenterLimelight.centerLimelightCenter(),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.acceptAlgae()
				),
				drivetrain.slowMode()
			)
		);
	}
	public Command telePositionForAlgaeOverideL2() {
		return new SequentialCommandGroup(
			this.elevator.setTargetAlgaeLevelCommand(AlgaePosition.L2),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.acceptAlgae()
				),
				drivetrain.slowMode()
			)
		);
	}
	public Command telePositionForAlgaeOverideL3() {
		return new SequentialCommandGroup(
			this.elevator.setTargetAlgaeLevelCommand(AlgaePosition.L3),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.acceptAlgae()
				),
				drivetrain.slowMode()
			)
		);
	}


	public Command pullAlgaeOffReef() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
			drivetrain.slowMode()
		).until(() -> 
			this.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.blueReefCenter) > Tuning.reefBackupWithAlgaeRadius.get() 
			&& this.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.redReefCenter) > Tuning.reefBackupWithAlgaeRadius.get()
		);
	}
	public Command passCoral(){
		return new SequentialCommandGroup(
			Commands.deadline(
				new SequentialCommandGroup(
					this.bananaFlywheels.outputForward().withTimeout(Units.Seconds.of(0.5)),
					new InstantCommand(() -> {elevator.onEjectCoral();}),
					new RunCommand(() -> {}).until(elevator::isInTargetPos)
				),
				this.intake.runTrough().until(intake::holdingGamePeice)
			),
			new ParallelCommandGroup(
				this.intake.runTrough(),
				this.bananaFlywheels.outputForward()
			).until(() -> !this.intake.holdingGamePeice())
		);
	}

	public String getDriveMode() { return this.driveModeChooser.get(); }
}
