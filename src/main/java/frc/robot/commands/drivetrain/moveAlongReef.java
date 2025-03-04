// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveAlongReef extends Command {
  /** Creates a new moveAlongReef. */
  public moveAlongReef() {
  }
  public final DriverOI oi = RobotContainer.getInstance().driverOI;
  private double slowedAmount = 2.0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getInstance().drivetrain.control(speeds());
  }

  private Translation2d translation() {
		// get inputs, apply deadbands
		double axial = this.oi.driveAxial.getAsDouble()/slowedAmount;
		double lateral = this.oi.driveLateral.getAsDouble()/slowedAmount;
		axial = -MathUtil.applyDeadband(axial, 0.25); // Negate b/c joystick Y is inverted from field X
		lateral = -MathUtil.applyDeadband(lateral, 0.25); // Negate b/c joystick X is inverted from field Y
		Logger.recordOutput("Drivetrain/JoystickDrive/Axial", axial);
		Logger.recordOutput("Drivetrain/JoystickDrive/Lateral", lateral);

		// Calculate the move magnitude
		final double magnitude = Math.hypot(axial, lateral); // get length and normalize
		final double desaturationFactor = Math.max(magnitude, 1.0); // guarantees the output is between -1 and 1

		// Convert to m/s
		final LinearVelocity vx = Constants.Drivetrain.maxVelocity.times(axial / desaturationFactor);
		final LinearVelocity vy = Constants.Drivetrain.maxVelocity.times(lateral / desaturationFactor);

		return new Translation2d(vx.in(Units.MetersPerSecond), vy.in(Units.MetersPerSecond));
	}
  public ChassisSpeeds speeds() {
		final Translation2d translation = this.translation();
		return new ChassisSpeeds(translation.getX(), 0,0);
	}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
