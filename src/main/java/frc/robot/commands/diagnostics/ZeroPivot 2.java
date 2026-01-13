package frc.robot.commands.diagnostics;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIOReal;

public class ZeroPivot extends InstantCommand {
	public ZeroPivot() {
		super(() -> {
			final CANcoder enc = ((ShooterIOReal) Robot.cont.shooter.io).encoder;
			final CANcoderConfiguration cfg = new CANcoderConfiguration();
			enc.getConfigurator().refresh(cfg);
			cfg.MagnetSensor.MagnetOffset = cfg.MagnetSensor.MagnetOffset
				- Robot.cont.shooter.inputs.angle.in(Units.Rotations);
			enc.getConfigurator().apply(cfg);
		});
	}
}
