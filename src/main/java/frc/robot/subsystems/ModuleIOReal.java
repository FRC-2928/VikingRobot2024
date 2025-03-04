// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule.Place;

public class ModuleIOReal implements ModuleIO {
	public final Place place;

	public final TalonFX drive;
	public final TalonFX azimuth;
	public final CANcoder cancoder;

	public final StatusSignal<Angle> drivePosition;
	public final StatusSignal<AngularVelocity> driveVelocity;
	public final StatusSignal<Current> driveCurrent;

	public final StatusSignal<Current> azimuthCurrent;

	public final StatusSignal<Angle> angle;

	public final Angle absoluteEncoderOffset;

	private final DCMotorSim driveMotor = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
		0.00001,
		Constants.Drivetrain.driveGearRatio), 
			DCMotor.getKrakenX60Foc(1)
		);
	private final DCMotorSim azimuthMotor = new DCMotorSim(
		LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
		0.00001,
		Constants.Drivetrain.azimuthGearRatio), 
		DCMotor.getKrakenX60Foc(1)
	);

	public ModuleIOReal(final SwerveModule module) {
		this.place = module.place;

		switch(this.place) {
		case FrontLeft:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveFrontLeftAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveFrontLeftDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveFrontLeftAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveFrontLeftOffset;
			break;
		case FrontRight:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveFrontRightAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveFrontRightDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveFrontRightAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveFrontRightOffset;
			break;
		case BackRight:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveBackRightAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveBackRightDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveBackRightAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveBackRightOffset;
			break;
		case BackLeft:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveBackLeftAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveBackLeftDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveBackLeftAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveBackLeftOffset;
			break;
		default:
			throw new RuntimeException("Invalid module index");
		}

		final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		
		driveConfig.MotorOutput.Inverted = (this.place == Place.FrontRight || this.place == Place.BackRight) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		// Peak output amps
		driveConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		driveConfig.Feedback.SensorToMechanismRatio =  Constants.Drivetrain.driveGearRatio/Constants.Drivetrain.wheelCircumference.in(Units.Meters);
		driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// PID values
		driveConfig.Slot0 = Slot0Configs.from(Constants.Drivetrain.drive);

		driveConfig.Audio = Constants.talonFXAudio;

		this.drive.getConfigurator().apply(driveConfig);
		this.drive.setNeutralMode(NeutralModeValue.Brake);

		final TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
		// Peak output of 40 amps
		azimuthConfig.CurrentLimits.StatorCurrentLimit = 40.0;
		azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		azimuthConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		azimuthConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		azimuthConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// Supply current limits
		azimuthConfig.CurrentLimits.SupplyCurrentLimit = 35;
		azimuthConfig.CurrentLimits.SupplyCurrentLimit = 60;  	   // max current draw allowed
		azimuthConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // maximum current allowed *after* the supply current limit is reached
		azimuthConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		azimuthConfig.Feedback.FeedbackRemoteSensorID = this.cancoder.getDeviceID();
		azimuthConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.azimuthGearRatio;

		azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;

		azimuthConfig.Slot0 = Slot0Configs.from(Constants.Drivetrain.azimuth);

		azimuthConfig.Audio = Constants.talonFXAudio;

		this.azimuth.getConfigurator().apply(azimuthConfig);
		this.azimuth.setNeutralMode(NeutralModeValue.Brake);

		final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
		magnetSensorConfigs.withAbsoluteSensorDiscontinuityPoint(0.5);  // TODO: make this magic number a constant
		encoderConfig.withMagnetSensor(magnetSensorConfigs);
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.MagnetOffset = this.absoluteEncoderOffset.in(Units.Rotations);
		this.cancoder.getConfigurator().apply(encoderConfig);

		this.drivePosition = this.drive.getRotorPosition();
		this.driveVelocity = this.drive.getVelocity();
		this.driveCurrent = this.drive.getStatorCurrent();
		this.azimuthCurrent = this.azimuth.getStatorCurrent();
		this.angle = this.cancoder.getAbsolutePosition();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.drivePosition, this.angle); // Required for odometry, use faster rate
		BaseStatusSignal
			.setUpdateFrequencyForAll(
				50,
				this.driveCurrent,

				this.azimuthCurrent
			);
		this.drive.optimizeBusUtilization();
		this.azimuth.optimizeBusUtilization();
	}

	@Override
	public void setDriveVoltage(final double volts) {
		this.drive.setControl(new VoltageOut(volts).withEnableFOC(
			RobotContainer.getInstance().operatorOI.foc.getAsBoolean() || DriverStation.isAutonomous()));
	}

	@Override
	public void drive(final LinearVelocity demand) {
		this.drive.setControl(new VelocityVoltage(demand.in(Units.MetersPerSecond)));
	}

	@Override
	public void azimuth(final Angle desired) {
		this.azimuth.setControl(new PositionVoltage(desired.in(Units.Rotations)));
	}

	@Override
	public void updateInputs(final ModuleIOInputs inputs) {
		BaseStatusSignal
			.refreshAll(
				this.drivePosition,
				this.driveVelocity,
				this.driveCurrent,
				this.azimuthCurrent,
				this.angle
			);

		inputs.drivePosition = this.drivePosition.getValue()
			.div(Constants.Drivetrain.driveGearRatio);
		inputs.driveVelocity = Units.MetersPerSecond
			.of(this.driveVelocity.getValueAsDouble());
		inputs.driveCurrent = this.driveCurrent.getValue();
		inputs.azimuthCurrent = this.azimuthCurrent.getValue();
		inputs.angle = this.angle.getValue();

		Logger.recordOutput("Drivetrain/" + this.place.name() + "/Angle", inputs.angle);
	}

	@Override
	public void updateSimulation(final double dt) {
		TalonFXSimState driveSim = this.drive.getSimState();
		TalonFXSimState azimuthSim = this.azimuth.getSimState();
		CANcoderSimState cancoderSim = this.cancoder.getSimState();

		driveSim.Orientation = (this.place == Place.FrontRight || this.place == Place.BackRight) ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

		driveSim.setSupplyVoltage(Units.Volts.of(12));
		azimuthSim.setSupplyVoltage(Units.Volts.of(12));
		cancoderSim.setSupplyVoltage(Units.Volts.of(12));

		driveMotor.setInputVoltage(addFrictionToVoltage(driveSim.getMotorVoltageMeasure().in(Units.Volts), 0.225));
		azimuthMotor.setInputVoltage(addFrictionToVoltage(azimuthSim.getMotorVoltageMeasure().in(Units.Volts), 0.225));
		driveMotor.update(dt);
		azimuthMotor.update(dt);
		
		driveSim.setRawRotorPosition(driveMotor.getAngularPosition().times(Constants.Drivetrain.driveGearRatio));
		driveSim.setRotorVelocity(driveMotor.getAngularVelocity().times(Constants.Drivetrain.driveGearRatio));
		cancoderSim.setRawPosition(azimuthMotor.getAngularPosition());
		cancoderSim.setVelocity(azimuthMotor.getAngularVelocity());
		azimuthSim.setRawRotorPosition(azimuthMotor.getAngularPositionRotations() * Constants.Drivetrain.azimuthGearRatio);
		azimuthSim.setRotorVelocity(azimuthMotor.getAngularVelocity().times(Constants.Drivetrain.azimuthGearRatio));
	}

	public double addFrictionToVoltage(double voltage, double friction) {
		if (voltage < 0) {
			return Math.min(voltage + friction, 0);
		}
		return Math.max(voltage - friction, 0);
	}
}
