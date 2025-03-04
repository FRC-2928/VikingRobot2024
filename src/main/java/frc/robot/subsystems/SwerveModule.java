package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class SwerveModule {
	public static enum Place {
		FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3);

		private Place(final int index) { this.index = index; }

		public final int index;
	}

	public SwerveModule(final Place place) {
		this.place = place;

		this.io = switch(Constants.mode) {
		case REAL -> new ModuleIOReal(this);
		case REPLAY -> new ModuleIO() {
		};
		case SIM -> new ModuleIOReal(this);
		default -> throw new Error();
		};
	}

	public final Place place;
	public final ModuleIO io;
	public final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

	public SwerveModulePosition position = new SwerveModulePosition();
	public SwerveModuleState current = new SwerveModuleState();
	public SwerveModuleState desired = new SwerveModuleState();

	public Distance drivePosition() {
		return Units.Meters
			.of(
				this.inputs.drivePosition.in(Units.Rotations) * Constants.Drivetrain.wheelCircumference.in(Units.Meters)
			);
	}

	public void halt() { this.io.setDriveVoltage(0); }

	public void control(final SwerveModuleState state) {
		if(Math.abs(state.angle.minus(new Rotation2d(this.inputs.angle)).getDegrees()) > 90) {
			state.speedMetersPerSecond = -state.speedMetersPerSecond;
			state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180.0));
		}

		this.io.azimuth(state.angle.getMeasure());
		this.io.drive(Units.MetersPerSecond.of(state.speedMetersPerSecond));

		this.desired = state;
	}

	public void runCharacterization(final double volts) {
		this.io.azimuth(Units.Degrees.of(0));
		this.io.setDriveVoltage(volts);
	}

	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Drivetrain/" + this.place.name(), this.inputs);

		this.position = new SwerveModulePosition(
			this.drivePosition().in(Units.Meters),
			new Rotation2d(this.inputs.angle)
		);
		this.current = new SwerveModuleState(
			this.inputs.driveVelocity.in(Units.MetersPerSecond),
			new Rotation2d(this.inputs.angle)
		);
	}

	public void simulationPeriodic() {
		this.io.updateSimulation(0.02);
	}
}
