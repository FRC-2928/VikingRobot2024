package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

import edu.wpi.first.units.Units;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public Angle angle = Units.Radians.zero();
		public AngularVelocity angleSpeed = Units.RadiansPerSecond.zero();
		public boolean holdingNote;
		public AngularVelocity flywheelSpeedA;
		public AngularVelocity flywheelSpeedB;
	}

	public static enum Demand {
		Reverse(-1), Halt(0), Forward(1);

		private Demand(final double dir) { this.dir = dir; }

		public final double dir;
	}

	public default void rotate(final Angle target) {}

	public default void runFlywheels(final double demand) {}

	public default void runFlywheelsVelocity(final double demand) {}

	public default void runFeeder(final Demand demand) {}

	public default void runIntake(final Demand demand) {}

	public default void extendAmpBar() {}

	public default void retractAmpBar() {}

	public default void updateInputs(final ShooterIOInputs inputs) {}
}
