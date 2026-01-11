package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.utils.STalonFX;

public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		final TalonFXConfiguration actuator = new TalonFXConfiguration();

		actuator.HardwareLimitSwitch.ReverseLimitEnable = true;
		actuator.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
		actuator.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
		actuator.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.CAN.CTRE.climber;
		actuator.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
		actuator.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
		actuator.Slot0 = Slot0Configs.from(Constants.Climber.configFast);
		actuator.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		actuator.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		actuator.Audio = Constants.talonFXAudio;

		this.actuator.getConfigurator().apply(actuator);

		this.actuator.setPosition(0);

		this.position = this.actuator.getPosition();
		this.home = this.actuator.getReverseLimit();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.position, this.home);
		this.actuator.optimizeBusUtilization();

		this.lock(true);
	}

	public final STalonFX actuator = new STalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);

	public final Servo lock = new Servo(9);

	public final StatusSignal<Angle> position;
	public final StatusSignal<ReverseLimitValue> home;

	private boolean disengaging;
	private double beginDisengagePosition;

	private double demand;

	@Override
	public void set(final double position) { this.demand = position; }

	@Override
	public void override(final double dutyCycle) {
		this.actuator.setControl(new DutyCycleOut(dutyCycle));

		this.demand = this.position.getValueAsDouble();
	}

	@Override
	public void offset(final double offset) { this.set(this.position.getValueAsDouble() + offset); }

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position, this.home);

		inputs.position = this.position.getValueAsDouble();
		inputs.home = this.home.getValue() == ReverseLimitValue.ClosedToGround;
	}

	@Override
	@SuppressWarnings("unused")
	public void periodic() {
		Logger.recordOutput("Climber/DriveUp", this.demand == 129);
		Logger.recordOutput("Climber/Disengaging", this.disengaging);
		Logger.recordOutput("Climber/RatchetLocked", this.lock.getAngle() == Constants.Climber.ratchetLocked);

		if(Math.abs(this.demand - this.position.getValueAsDouble()) < 0.1 && Constants.Climber.ratchetEnabled) {
			this.lock(true);
			this.control(this.position.getValueAsDouble());
		} else if(this.demand - 0.1 <= this.position.getValueAsDouble() || !Constants.Climber.ratchetEnabled) {
			this.lock(false);
			this.control(this.demand);

			this.disengaging = false;

			Logger.recordOutput("Climber/State", "Eng");
		} else {
			if(this.disengaging) {
				if(
					this.position.getValueAsDouble() < this.beginDisengagePosition - Constants.Climber.disengageDistance
						|| this.home.getValue() == ReverseLimitValue.ClosedToGround
				) {
					this.lock(false);
					this.control(this.demand);

					this.disengaging = false;

					Logger.recordOutput("Climber/State", "DisFin");
				} else {
					this.lock(false);
					this.control(this.beginDisengagePosition - Constants.Climber.disengageDistance * 2);

					Logger.recordOutput("Climber/State", "DisMove");
				}
			} else {
				if(this.lock.getAngle() == Constants.Climber.ratchetLocked) {
					this.lock(false);
					this.control(this.beginDisengagePosition - Constants.Climber.disengageDistance * 2);

					this.disengaging = true;
					this.beginDisengagePosition = this.position.getValueAsDouble();

					Logger.recordOutput("Climber/State", "BegDis");
				} else {
					this.lock(false);
					this.control(this.demand);

					Logger.recordOutput("Climber/State", "AlreadyDis");
				}
			}
		}
	}

	private void control(final double target) {
		this.actuator.setControl(new PositionDutyCycle(target));
		Logger.recordOutput("Climber/TargetPosition", target);
	}

	private void lock(final boolean engaged) {
		this.lock.setAngle(engaged ? Constants.Climber.ratchetLocked : Constants.Climber.ratchetFree);
		Logger.recordOutput("Climber/LockEngaged", engaged);
	}
}
