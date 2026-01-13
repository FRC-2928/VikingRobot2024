// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Measure; 
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LookForNote extends Command {
  /** Creates a new lookForNote. */
  public LookForNote(Angle rotAmount) { 
    this.addRequirements(Robot.cont.drivetrain); 
    this.rotationAmount = rotAmount;
  }

  private Angle initalAngle;
  private Angle currentAngle;
  private final Angle rotationAmount;
  private Angle setpoint;
  private double computedPidValue;
  private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
		.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations()); 
    this.initalAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations());
    this.absoluteController.enableContinuousInput(-0.5,0.5);
    this.setpoint = this.initalAngle.plus(this.rotationAmount);
    this.absoluteController.reset(this.initalAngle.in(Units.Rotations));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations()); 
    double measurement = this.currentAngle.in(Units.Rotations);
    this.computedPidValue = -this.absoluteController.calculate(measurement,this.setpoint.in(Units.Rotations));
    AngularVelocity rotationSpeed = Constants.Drivetrain.maxAngularVelocity.times( MathUtil.applyDeadband(this.computedPidValue,0.008));
    Robot.cont.drivetrain
      .control(
        Robot.cont.drivetrain
          .rod(
          new ChassisSpeeds(
            0,
            0,
            rotationSpeed.in(Units.RadiansPerSecond)
            )
          )
      );
    Logger.recordOutput("Drivetrain/Auto/setpoint",(this.setpoint.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/currentAngle",(this.currentAngle.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/initialAngle",(this.initalAngle.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/PIDvalue", this.computedPidValue);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
      Robot.cont.drivetrain.halt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hasRotatedThruAngle;
    double amountRotated = Math.abs(this.currentAngle.minus(this.initalAngle).in(Units.Radians));
    hasRotatedThruAngle = amountRotated >=  Math.abs(this.rotationAmount.in(Units.Radians)) - 0.03;
    Logger.recordOutput("Drivetrain/Auto/amountRotated",amountRotated);
    Logger.recordOutput("Drivetrain/Auto/hasRotatedThruAngle",(hasRotatedThruAngle));
    return Robot.cont.drivetrain.limelightNote.hasValidTargets() || (hasRotatedThruAngle && (Math.abs(this.currentAngle.minus(this.setpoint).in(Units.Radians)) < 0.03 && this.computedPidValue < 0.1));
  }
}
