package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Shoot extends Command {
    public Shoot() { this.addRequirements(Robot.instance.container.shooter); }

    @Override
    public void initialize() { Robot.instance.container.driverOI.hid.setRumble(RumbleType.kBothRumble, 0.25); }

    @Override
    public void execute() {
        Robot.instance.container.shooter.flywheels.set(1);
        Logger.recordOutput("Shooter/Velocity", Robot.instance.container.shooter.flywheels.getVelocity().getValue());
        if(Robot.instance.container.shooter.flywheels.getVelocity().getValueAsDouble() > 85) {
            Robot.instance.container.shooter.launcher.set(ControlMode.PercentOutput, 1);
            Robot.instance.container.driverOI.hid.setRumble(RumbleType.kBothRumble, 1);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        Robot.instance.container.shooter.flywheels.stopMotor();
        Robot.instance.container.shooter.launcher.set(ControlMode.Disabled, 0);
        Robot.instance.container.driverOI.hid.setRumble(RumbleType.kBothRumble, 0);
    }
}
