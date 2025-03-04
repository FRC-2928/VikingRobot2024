package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/*
 * This class instantiates LoggedNetworkNumbers for any tuning parameters we want to tune/change during runtime.
 * The values can be used in a class by calling number.get() in a subsystem periodic() or command execute()
 * and can be accessed/modified via Shuffleboard
 */
public final class Tuning {
	private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

	public static final LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber
		("Tuning/SpeedIntakePercent",
		.8);
	public static final LoggedNetworkNumber alignRadiusReef = new LoggedNetworkNumber
		("Tuning/AlignReefRadius",
		2);
	public static final LoggedNetworkNumber alignRadiusHP = new LoggedNetworkNumber
		("Tuning/AlignHPRadius",
		1.5);
	public static final LoggedNetworkNumber alignRadiusProcessor = new LoggedNetworkNumber
		("Tuning/AlignProcessorRadius",
		1.5);

	// How far away does the robot have to be from reef center before the elevator can go down with algae?
	public static final LoggedNetworkNumber reefBackupWithAlgaeRadius = new LoggedNetworkNumber
		("Tuning/ReefBackupWithAlgaeRadius",
		2);
	public static final LoggedNetworkNumber lFourHeightMeter = new LoggedNetworkNumber(
		"/Tuning/LFourHeight",
		1
	);
	public static final LoggedNetworkNumber noneHeightHome = new LoggedNetworkNumber(
		"/Tuning/NoneHeightHome",
		0
	);
	public static final LoggedNetworkNumber nonePivotHome = new LoggedNetworkNumber(
		"/Tuning/NonePivotHome",
		0
	);
	public static final LoggedNetworkNumber coralHeightHome = new LoggedNetworkNumber(
		"/Tuning/CoralHeightHome",
		0
	);
	public static final LoggedNetworkNumber coralPivotHome = new LoggedNetworkNumber(
		"/Tuning/CoralPivotHome",
		0
	);
	public static final LoggedNetworkNumber algaeHeightHome = new LoggedNetworkNumber(
		"/Tuning/AlgaeHeightHome",
		0.5
	);
	public static final LoggedNetworkNumber algaePivotHome = new LoggedNetworkNumber(
		"/Tuning/AlgaePivotHome",
		0
	);
	public static final LoggedNetworkNumber cageHeightHome = new LoggedNetworkNumber(
		"/Tuning/CageHeightHome",
		0
	);
	public static final LoggedNetworkNumber cagePivotHome = new LoggedNetworkNumber(
		"/Tuning/CagePivotHome",
		0
	);
	public static final LoggedNetworkNumber offsetCenterReef = new LoggedNetworkNumber(
		"/Tuning/offsetCenterReef",
		6.5
	);
	public static final LoggedNetworkNumber elevatorSpeed = new LoggedNetworkNumber(
		"/Tuning/elevatorSpeed",
		0.3
	);
}
