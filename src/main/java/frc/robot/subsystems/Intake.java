package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//  Kracken x60 for deploy retract w/ encoder
//  Kracken x60 for motors (all one)
// Minion for Belt
// Minion for trough and limit switch
public class Intake extends SubsystemBase {
	public static enum Feeder {
		Reverse(-1), Halt(0), Forward(1);

		public double voltage; // Stores the voltage commanded for that demand

		private Feeder(final double input) {
			this.voltage = input;
		}
	}
	@AutoLog
	public static class IntakeInputs {
		public boolean troughHasCoral;
		// public AngularVelocity intakeSpeed;
		// public Angle pivotAngle;
		public AngularVelocity troughSpeed;
	}

	public final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	// private final TalonFX wheels;
	// private final TalonFX pivot;
	private final TalonFXS trough;
	
	// private final StatusSignal<AngularVelocity> intakeSpeed;
	// private final StatusSignal<Angle> pivotAngle;
	// private TalonFXConfiguration intakeWheelsConfig = new TalonFXConfiguration();
	// private TalonFXConfiguration pivotWheelConfig = new TalonFXConfiguration();
	private final StatusSignal<ForwardLimitValue> troughSensor;
	private final StatusSignal<AngularVelocity> troughSpeed;

	public Intake(){
		trough = new TalonFXS(Constants.CAN.CTRE.troughWheels, Constants.CAN.CTRE.bus);
		// Set Neutral Mode
		trough.setNeutralMode(NeutralModeValue.Brake);

		// Create the Config Object for this TalonFXS
		TalonFXSConfiguration troughConfig = new TalonFXSConfiguration();

		// Motor Arrangement
		troughConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

		// Peak output amps
		troughConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		troughConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		// Supply current limits
		troughConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		troughConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		troughConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		troughConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit
		troughConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// Hardware Limit Switch Config
		troughConfig.HardwareLimitSwitch
			.withForwardLimitType(ForwardLimitTypeValue.NormallyClosed)
			.withForwardLimitEnable(true)
			.withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin);

		// // Ground Intake Config
		// // Ground Intake Wheels Config
		// intakeWheelsConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		// intakeWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		// intakeWheelsConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		// intakeWheelsConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		// // Supply current limits
		// intakeWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		// intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		// intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit
		// intakeWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
		// intakeWheelsConfig.Audio = Constants.talonFXAudio;
		// pivotWheelConfig = intakeWheelsConfig;
		
		// pivotWheelConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// pivotWheelConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.max.in(Units.Rotations);
		// pivotWheelConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// pivotWheelConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.pivotMin.in(Units.Rotations);
		// // PID values
		// pivotWheelConfig.Slot0 =  Constants.Intake.pivotConfig;
		// intakeWheelsConfig.Slot0 = Constants.Intake.flywheelGainsSlot0;

		// this.wheels.getConfigurator().apply(intakeWheelsConfig);
		// this.wheels.setNeutralMode(NeutralModeValue.Brake);
		// this.pivot.getConfigurator().apply(pivotWheelConfig);
		// this.pivot.setNeutralMode(NeutralModeValue.Brake);
		this.trough.getConfigurator().apply(troughConfig);

		// this.intakeSpeed = this.wheels.getRotorVelocity();
		// this.pivotAngle = this.pivot.getRotorPosition();
		this.troughSpeed = this.trough.getVelocity();
		this.troughSensor = this.trough.getForwardLimit();

		StatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(50), 
			troughSpeed,
			troughSensor
		);
	}
	
	public void runIntake(final Feeder demand){
		// Intake isn't integrated, do nothing
		// this.wheels.setControl(new VelocityDutyCycle(demand.demand*0.1));
	}

	public boolean holdingGamePeice() {
		return inputs.troughHasCoral;
	}

	private void runTrough(Feeder demand){
		this.trough.setControl(new VoltageOut(demand.voltage));
	}

	public void updateInputs(final IntakeInputs inputs) {
		BaseStatusSignal.refreshAll(this.troughSensor, this.troughSpeed);
		inputs.troughSpeed = this.troughSpeed.getValue();
	 	inputs.troughHasCoral = this.troughSensor.getValue().value != 0;
	}

	// Runs the trough until the command is interrupted, then stops the trough
	public Command runTrough() {
		return new RunCommand(() -> {
			runTrough(Feeder.Forward);
		}, this).finallyDo(
			() -> runTrough(Feeder.Halt)
		);
	}

	@Override
	public void periodic() {
		updateInputs(this.inputs);
	}
}
