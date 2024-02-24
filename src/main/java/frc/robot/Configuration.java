package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Configuration {
	static {
		SmartDashboard.setPersistent("Configuration/FieldOriented");
	}

	public static LoggedDashboardBoolean fod = new LoggedDashboardBoolean("Configuration/FieldOriented", true);
	public static LoggedDashboardBoolean absoluteRotation = new LoggedDashboardBoolean(
		"Configuration/AbsoluteRotation",
		true
	);
}