package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S2StateValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Banana;
import frc.robot.Constants.Banana.FeederDemand;
import frc.robot.Constants.GamePieceType;

/* 1.Make a method that stops elevator pivot from going lower a certain angle
 * 2. Run fly wheels forward and backward TalonFX
 * 3. Detect coral in banana - Kraken x44, limit switch
 * 	Start spinning flywheel when preparing intake from trough
 * 	Limit switches (detect presence or absence of): elevator can move with coral, coral in trough
 * States: 
 * true, true = coral being intook into banana
 * false, false = no coral in trough or banana
 * true, false = coral in banana
 * false, true = coral in trough
 * if banana doesn't have coral than banana shouldn't move past a certain angle/height
 * 
*/
public class BananaFlywheels extends SubsystemBase {
	@AutoLog
	public static class BananaFlywheelsInputs {
		public GamePieceType heldGamePieceType = GamePieceType.NONE;
		public GamePieceType targetedGamePieceType = GamePieceType.NONE;
		public Angle flywheelPosition = Units.Rotations.zero();
		public AngularVelocity flywheelSpeed = Units.RotationsPerSecond.zero();
		public Current flywheelStatorCurrent = Units.Amps.zero();
		public Current flywheelSupplyCurrent = Units.Amps.zero();
	}

	private final BananaFlywheelsInputsAutoLogged inputs = new BananaFlywheelsInputsAutoLogged();
	/// State tracking which @c GamePieceType is currently held by the Banana
	private GamePieceType heldGamePieceType;
	/// State tracking which @c GamePieceType is currently targeted by the Banana
	private GamePieceType targetedGamePieceType;

	private final TalonFX wheels;
	private final StatusSignal<Angle> angle;
	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Current> motorStatorCurrent;
	private final StatusSignal<Current> motorSupplyCurrent;
	private final StatusSignal<S2StateValue> beamBreakStateSignal;

	/**
	 * Default Constructor
	 */
	public BananaFlywheels() {
		this.heldGamePieceType = GamePieceType.NONE;
		this.targetedGamePieceType = GamePieceType.NONE;
		// S2 on the CANdi is tied to the beam-break sensor
		this.beamBreakStateSignal = Constants.CAN.RIO.BANANA_CANDI.getInstance().getS2State();
		this.wheels = new TalonFX(Constants.CAN.RIO.bananaWheels, Constants.CAN.RIO.bus);
		TalonFXConfiguration flyWheelConfig = new TalonFXConfiguration();

		flyWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Gearing, clockwise moves elevator up
		flyWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Peak output amps
		flyWheelConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		flyWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flyWheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		flyWheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		flyWheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flyWheelConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		flyWheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		flyWheelConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		flyWheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// PID values
		flyWheelConfig.Slot0 = Constants.Banana.flywheelGainsSlot0;
		flyWheelConfig.Audio = Constants.talonFXAudio;

		this.wheels.getConfigurator().apply(flyWheelConfig);

		this.angle = this.wheels.getRotorPosition();
		this.velocity = this.wheels.getRotorVelocity();
		this.motorStatorCurrent = this.wheels.getStatorCurrent();
		this.motorSupplyCurrent = this.wheels.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100),
			/* CANdi excluded because the singleton controls the update frequency */
			this.angle,
			this.velocity,
			this.motorStatorCurrent,
			this.motorSupplyCurrent);

		this.wheels.optimizeBusUtilization();
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs("BananaFlywheels", inputs);
		updateMotors();
	}

	// Control the motors based on the internal state machine
	private void updateMotors() {
		// TODO: implement
	}
	
	public void runFlywheels(FeederDemand demand) {
		this.wheels.setControl(new VoltageOut(demand.getValue()*6.0));
	}

	//Gets angle of flywheel and holds it at that position
	//Will add the holding coral input once we know which sensor detects the coral
	public void holdPosition() {
		Angle wheelAngle = this.angle.getValue();
		this.wheels.setControl(new PositionVoltage(wheelAngle));
	}

	public boolean holdingCoral() {
		return (heldGamePieceType == GamePieceType.CORAL);
	}

	public GamePieceType getHeldGamePieceType() {
		return heldGamePieceType;
	}

	public void setTargetGamePiece(GamePieceType targetPieceType) {
		this.targetedGamePieceType = targetPieceType;
	}

	private void updateInputs(final BananaFlywheelsInputs inputs) {
		BaseStatusSignal.refreshAll(
			this.angle, this.velocity, this.motorStatorCurrent, this.motorSupplyCurrent, this.beamBreakStateSignal);

		inputs.flywheelSpeed = velocity.getValue();
		inputs.flywheelSupplyCurrent = motorStatorCurrent.getValue();
		inputs.flywheelStatorCurrent = motorSupplyCurrent.getValue();
		inputs.targetedGamePieceType = this.targetedGamePieceType;

		// The held piece type can be expressed as a bitset of the various indicator flags
		boolean[] heldGamePieceTypeFlags = new boolean[2];  // create the array of boolean flags (bitflags)
		// In a proper bitmask the order of these fields wouldn't matter; however since our bitset is implicitly
		// tied to known values of the GamePieceType enum the order IS IMPORTANT
		// In binary: 00 = NONE, 01 = CORAL, 10 = ALGAE, 11 = CAGE
		heldGamePieceTypeFlags[0] = (beamBreakStateSignal.getValue() == S2StateValue.Low);  // Coral is bit 0
		heldGamePieceTypeFlags[1] = motorStatorCurrent.getValue().gt(Banana.HOLDING_ALGAE_CURRENT_THRESHOLD); // Algae is bit 1
		int heldGamePieceTypeBitset = 0;
		for (int i = 0; i < heldGamePieceTypeFlags.length; i++) {
			if (heldGamePieceTypeFlags[i]) {
				heldGamePieceTypeBitset |= (1 << i);  // shift the bits into position
			}
		}

		// extract the piece type enum from the integer (bitfield)
		this.heldGamePieceType = GamePieceType.fromInt(heldGamePieceTypeBitset);
		inputs.heldGamePieceType = this.heldGamePieceType;
	}

	/**
	 * Creates a command that will output the coral, then stop when it's out of the banana.
	 * This factory should be used for autonomous or for fully automating coral scoring.
	 * @return a command to score the coral
	 */
	public Command scoreHeldCoral() {
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.FORWARD);
		}, this).until(() -> !this.holdingCoral())
		.andThen(
			new RunCommand(() -> {
				runFlywheels(FeederDemand.FORWARD);
			}, this).withTimeout(0.2)
		).andThen(
			new RunCommand(() -> {
				runFlywheels(FeederDemand.HALT);
			},this)
		);
		// 1. Run the wheels forward until the sensor returns false
		// 2. Run the wheels another 0.2 seconds
		// 3. Stop the wheels
	}

	public Command outputForward(){
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.FORWARD);
		}, this).finallyDo(() -> {
			runFlywheels(FeederDemand.HALT);
		});
	}

	public Command acceptAlgae(){
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.REVERSE);
		}, this).finallyDo(this::holdPosition);
	}
}
