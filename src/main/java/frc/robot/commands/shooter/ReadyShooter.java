// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Tuning;

public class ReadyShooter extends Command {
	public ReadyShooter(final Angle angle, final boolean spinUp) {
		this.angle = angle;
		this.spinUp = spinUp;
		this.addRequirements(Robot.cont.shooter);
	}

	public final Angle angle;
	public final boolean spinUp;

	@Override
	public void initialize() { Robot.cont.shooter.io.retractAmpBar(); }

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(this.angle);
		if(this.spinUp) Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - this.angle.in(Units.Degrees)) < 2;
	}
}
