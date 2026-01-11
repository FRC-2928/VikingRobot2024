package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.utils.STalonFX;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal(final Shooter shooter) {
		this.angle = this.encoder.getAbsolutePosition();
		this.angleSpeed = this.encoder.getVelocity();
		this.velocityA = this.flywheelA.getRotorVelocity();
		this.velocityB = this.flywheelB.getRotorVelocity();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.angle, this.velocityA, this.velocityB);
		this.pivot.optimizeBusUtilization();
		this.encoder.optimizeBusUtilization();
		this.flywheelA.optimizeBusUtilization();
		this.flywheelB.optimizeBusUtilization();

		this.sensors = this.feeder.getSensorCollection();

		final TalonFXConfiguration pivot = new TalonFXConfiguration();

		pivot.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.pivotCurrentLimit;
		pivot.CurrentLimits.SupplyCurrentLimitEnable = true;

		pivot.Feedback.FeedbackRemoteSensorID = Constants.CAN.CTRE.shooterPivot;
		pivot.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

		pivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.max.in(Units.Rotations);
		pivot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Shooter.intakeGround.in(Units.Rotations);

		pivot.Slot0 = Slot0Configs.from(Constants.Shooter.pivotConfig);

		pivot.Audio = Constants.talonFXAudio;

		this.pivot.getConfigurator().apply(pivot);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);
		// this.pivot.setInverted(false);

		final TalonFXConfiguration flywheels = new TalonFXConfiguration();
		flywheels.Slot0 = Constants.Shooter.flywheelGainsSlot0;
		flywheels.Audio = Constants.talonFXAudio;

		this.flywheelA.getConfigurator().apply(flywheels);
		this.flywheelA.setNeutralMode(NeutralModeValue.Coast);
		// this.flywheelA.setInverted(false);

		this.flywheelB.getConfigurator().apply(flywheels);
		this.flywheelB.setNeutralMode(NeutralModeValue.Coast);
		// this.flywheelB.setInverted(true);
		//this.flywheelB.setControl(new Follower(this.flywheelA.getDeviceID(), true));

		this.feeder.setNeutralMode(NeutralMode.Brake);
		this.feeder.setInverted(true);
		this.intake.setNeutralMode(NeutralMode.Brake);
		this.intake.setInverted(true);

		this.sysIdPivot = new SysIdRoutine(
			new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(1), Units.Volts.of(7), null, state -> {
				Logger.recordOutput("SysId/Shooter/Pivot/State", state.toString());
				//this.sysIdPivot.recordState(state);

				BaseStatusSignal
					.setUpdateFrequencyForAll(
						state != State.kNone ? 100 : 0, // don't update unless currently running sysid
						this.pivot.getMotorVoltage(),
						this.pivot.getPosition(),
						this.pivot.getVelocity()
					);
			}),
			new SysIdRoutine.Mechanism(
				voltage -> this.pivot.setControl(new VoltageOut(voltage.in(Units.Volts))),
				log -> {
					//*
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Voltage",
							Units.Volts.of(this.pivot.getSupplyVoltage().getValueAsDouble())
						);
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Position",
							Units.Rotations.of(this.pivot.getPosition().getValueAsDouble())
						);
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Velocity",
							Units.RotationsPerSecond.of(this.pivot.getVelocity().getValueAsDouble())
						);
				},
				shooter
			)
		);

		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Quasistatic/Forward",
				this.sysIdPivot
					.quasistatic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Quasistatic/Reverse",
				this.sysIdPivot
					.quasistatic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intakeGround
							.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Dynamic/Forward",
				this.sysIdPivot
					.dynamic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Dynamic/Reverse",
				this.sysIdPivot
					.dynamic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intakeGround
							.in(Units.Rotations)
					)
			);
	}

	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final CANcoder encoder = new CANcoder(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final STalonFX flywheelA = new STalonFX(Constants.CAN.CTRE.shooterFlywheelA, Constants.CAN.CTRE.bus);
	public final STalonFX flywheelB = new STalonFX(Constants.CAN.CTRE.shooterFlywheelB, Constants.CAN.CTRE.bus);
	public final TalonSRX feeder = new TalonSRX(Constants.CAN.Misc.feederLauncher);
	public final TalonSRX intake = new TalonSRX(Constants.CAN.Misc.intakeRoller);
	private final Servo ampServoA = new Servo(Constants.PWM.ampBarServoA);
	private final Servo ampServoB = new Servo(Constants.PWM.ampBarServoB);

	public final StatusSignal<Angle> angle;
	public final StatusSignal<AngularVelocity> angleSpeed;
	public final StatusSignal<AngularVelocity> velocityA;
	public final StatusSignal<AngularVelocity> velocityB;
	public final SensorCollection sensors;

	public final SysIdRoutine sysIdPivot;

	@Override
	public void rotate(final Angle target) {
		final Angle rot = Units.Radians
			.of(
				MathUtil
					.clamp(
						target.in(Units.Radians),
						Constants.Shooter.intakeGround.in(Units.Radians),
						Constants.Shooter.max.in(Units.Radians)
					)
			);
		Logger.recordOutput("Shooter/PivotControl", rot);
		Logger
			.recordOutput(
				"Shooter/AngleDifference",
				rot.minus(Units.Rotations.of(this.angle.getValueAsDouble())).in(Units.Degrees)
			);
		this.pivot.setControl(new PositionDutyCycle(rot.in(Units.Rotations)));
	}

	@Override
	public void runFlywheelsVelocity(final double demand) {
		this.flywheelA.setControl(new VelocityDutyCycle(demand));
		this.flywheelB.setControl(new VelocityDutyCycle(demand));
	}

	@Override
	public void runFlywheels(final double demand) {
		this.flywheelA.setControl(new DutyCycleOut(demand));
		this.flywheelB.setControl(new DutyCycleOut(demand));

	}

	@Override
	public void runFeeder(final Demand demand) {
		this.feeder.set(ControlMode.PercentOutput, demand.dir);
		Logger.recordOutput("Shooter/FeederDemand", demand.name());
	}

	@Override
	public void runIntake(final Demand demand) {
		this.intake.set(ControlMode.PercentOutput, demand.dir);
		Logger.recordOutput("Shooter/IntakeDemand", demand.name());
	}

	@Override
	public void extendAmpBar() {
		this.ampServoA.set(Constants.Shooter.ampBarServoAExtend);
		this.ampServoB.set(Constants.Shooter.ampBarServoBExtend);
	}

	@Override
	public void retractAmpBar() {
		this.ampServoA.set(Constants.Shooter.ampBarServoARetract);
		this.ampServoB.set(Constants.Shooter.ampBarServoBRetract);
	}

	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.angle, this.velocityA);

		inputs.angle = Units.Rotations.of(this.angle.getValueAsDouble());
		inputs.angleSpeed = Units.RotationsPerSecond.of(this.angleSpeed.getValueAsDouble());
		inputs.flywheelSpeedA = Units.RotationsPerSecond.of(this.velocityA.getValueAsDouble());
		inputs.flywheelSpeedB = Units.RotationsPerSecond.of(this.velocityB.getValueAsDouble());
		inputs.holdingNote = !this.sensors.isFwdLimitSwitchClosed();
	}
}
