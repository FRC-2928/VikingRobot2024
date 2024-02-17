package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Intake extends Command {
    public Intake() { this.addRequirements(Robot.instance.container.shooter); }

    @Override
    public void execute() {
        if(Robot.instance.container.shooter.launcher.getSensorCollection().isFwdLimitSwitchClosed()) {
            Robot.instance.container.shooter.flywheels.set(-0.75);
            Robot.instance.container.shooter.launcher.set(ControlMode.PercentOutput, -0.15);
            Robot.instance.container.driverOI.hid.setRumble(RumbleType.kBothRumble, 0.25);
        } else {
            Robot.instance.container.shooter.flywheels.stopMotor();
            Robot.instance.container.shooter.launcher.set(ControlMode.PercentOutput, 0);
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
