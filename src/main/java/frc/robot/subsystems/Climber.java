package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
	public Climber() {
		this.io = switch(Constants.mode) {
		case REAL -> new ClimberIOReal();
		default -> throw new Error();
		};
	}

	public final ClimberIO io;
	public final ClimberIOInputs inputs = new ClimberIOInputs(){};

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		//Logger.processInputs("Climber", this.inputs);

		this.io.periodic();
	}
}
