package frc.robot.subsystems;

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
import frc.robot.Constants;
import frc.robot.utils.STalonFX;
import frc.robot.utils.SignalBundle;

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

		this.mSignalBundle = new SignalBundle<>(
			new BaseStatusSignal[] { this.position, this.home },
			inputs -> {
				inputs.position = this.position.getValueAsDouble();
				inputs.home = this.home.getValue() == ReverseLimitValue.ClosedToGround;
			}
		);
	}

	public final STalonFX actuator = new STalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);

	public final StatusSignal<Angle> position;
	public final StatusSignal<ReverseLimitValue> home;

	private final SignalBundle<ClimberIOInputs> mSignalBundle;

	@Override
	public void set(final double position) {
		this.actuator.setControl(new PositionDutyCycle(position));
	}

	@Override
	public void override(final double dutyCycle) {
		this.actuator.setControl(new DutyCycleOut(dutyCycle));
	}

	@Override
	public void offset(final double offset) {
		this.set(this.position.getValueAsDouble() + offset);
	}

	@Override
	public BaseStatusSignal[] getStatusSignals() {
		return mSignalBundle.signals();
	}

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		mSignalBundle.update(inputs);
	}

}
