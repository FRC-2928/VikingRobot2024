package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

/**
 * Hardware configuration for the swerve drivetrain.
 * Moved from the repo root and migrated from the deprecated
 * com.ctre.phoenix6.mechanisms.swerve package to com.ctre.phoenix6.swerve.
 *
 * CANcoder offsets come from physical calibration — preserve these exactly.
 */
public class TunerConstants {

    // --- Steer motor gains (from Tuner X calibration) ---
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);

    // --- Drive motor gains (tuned on robot) ---
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(4).withKI(0).withKD(0.2)
        .withKS(0.225).withKV(2.62).withKA(0);

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Stator current at which wheels start to slip
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12 V
    public static final double kSpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;  // SDS MK4i L2
    private static final double kSteerGearRatio = 21.428571428571427; // 150/7

    // Wheel radius in inches
    private static final double kWheelRadiusInches = 2.0;

    // Drive inversion: left side normal, right side inverted
    private static final boolean kInvertLeftSide  = false;
    private static final boolean kInvertRightSide = true;

    // Steer and encoder inversion (same for all modules)
    private static final boolean kSteerMotorReversed = false;
    private static final boolean kEncoderReversed    = false;

    private static final String kCANbusName = "canivore";
    private static final int    kPigeonId   = 0;

    // Simulation parameters
    private static final double kSteerInertia        = 0.00001;
    private static final double kDriveInertia        = 0.001;
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    // --- Drivetrain-level constants ---
    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANBusName(kCANbusName);

    // --- Module factory ---
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(Units.inchesToMeters(kWheelRadiusInches))
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12Volts(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio);

    // --- Front Left ---
    private static final int    kFrontLeftDriveMotorId  = 14;
    private static final int    kFrontLeftSteerMotorId  = 13;
    private static final int    kFrontLeftEncoderId     = 13;
    private static final double kFrontLeftEncoderOffset = -0.42138671875; // rotations
    private static final double kFrontLeftXPosInches    = 12;
    private static final double kFrontLeftYPosInches    = 12;

    // --- Front Right ---
    private static final int    kFrontRightDriveMotorId  = 18;
    private static final int    kFrontRightSteerMotorId  = 19;
    private static final int    kFrontRightEncoderId     = 19;
    private static final double kFrontRightEncoderOffset = 0.2978515625; // rotations
    private static final double kFrontRightXPosInches    = 12;
    private static final double kFrontRightYPosInches    = -12;

    // --- Back Left ---
    private static final int    kBackLeftDriveMotorId  = 11;
    private static final int    kBackLeftSteerMotorId  = 10;
    private static final int    kBackLeftEncoderId     = 10;
    private static final double kBackLeftEncoderOffset = 0.027587890625; // rotations
    private static final double kBackLeftXPosInches    = -12;
    private static final double kBackLeftYPosInches    = 12;

    // --- Back Right ---
    private static final int    kBackRightDriveMotorId  = 16;
    private static final int    kBackRightSteerMotorId  = 15;
    private static final int    kBackRightEncoderId     = 15;
    private static final double kBackRightEncoderOffset = -0.4169921875; // rotations
    private static final double kBackRightXPosInches    = -12;
    private static final double kBackRightYPosInches    = -12;

    // --- Module constants (passed to DriveSubsystem constructor) ---
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,  kFrontLeftDriveMotorId,  kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),  Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide, kSteerMotorReversed, kEncoderReversed);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide, kSteerMotorReversed, kEncoderReversed);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,   kBackLeftDriveMotorId,   kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches),  Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide, kSteerMotorReversed, kEncoderReversed);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,  kBackRightDriveMotorId,  kBackRightEncoderId,
            kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide, kSteerMotorReversed, kEncoderReversed);
}
