package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public class ClimberIOInputs {
		public double position;
		public boolean home;
	}

	public default void set(final double position) {}

	public default void override(final double dutyCycle) {}

	public default void offset(final double offset) {}

	public default void updateInputs(final ClimberIOInputs inputs) {}

	public default BaseStatusSignal[] getStatusSignals() { return new BaseStatusSignal[0]; }
}
