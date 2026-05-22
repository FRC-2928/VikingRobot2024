package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.vision.Limelight;

/**
 * Swerve drivetrain subsystem backed by CTRE's SwerveDrivetrain.
 *
 * Architecture role: receives a DriveGoal from the Superstructure (future) or
 * from commands directly (current). Handles all low-level module control,
 * odometry, and vision fusion internally.
 *
 * Data flow:
 *   applyGoal(DriveGoal) → sets wantedState
 *   periodic() → handleStateTransition() → applies the correct SwerveRequest
 */
public class DriveSubsystem
    extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {

    // ── Goal-based architecture scaffolding ─────────────────────────────────
    enum WantedState { TELEOP, LOCK, AUTONOMOUS }
    enum SystemState { TELEOP, LOCK, AUTONOMOUS }

    private WantedState wantedState = WantedState.TELEOP;
    private SystemState systemState = SystemState.TELEOP;

    // ── CTRE SwerveRequests (allocate once, mutate via with-methods) ─────────
    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    // ── Vision standard deviations (MegaTag2: trust X/Y, ignore yaw from cam) ─
    private static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.7, 0.7, 9999999);

    // ── Vision/Limelight ─────────────────────────────────────────────────────
    public final Limelight limelightNote    = new Limelight("limelight-note");
    public final Limelight limelightShooter = new Limelight("limelight-shooter");
    public final Limelight limelightRear    = new Limelight("limelight-rear");

    // ── Teleop joystick command (held here so periodic can cache its speeds) ──
    public final JoystickDrive joystickDrive;

    /**
     * Cached joystick chassis speeds, updated every periodic cycle.
     * Public so that commands like IntakeGround can read and add corrections.
     */
    public ChassisSpeeds joystickSpeeds = new ChassisSpeeds();

    // ── Constructor ──────────────────────────────────────────────────────────
    public DriveSubsystem() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        // Register with the WPILib command scheduler (required when not extending SubsystemBase)
        CommandScheduler.getInstance().registerSubsystem(this);

        // Configure vision trust level once
        setVisionMeasurementStdDevs(VISION_STD_DEVS);

        // Configure PathPlanner AutoBuilder
        final RobotConfig robotConfig = Constants.Drivetrain.Auto.config;
        AutoBuilder.configure(
            () -> getState().Pose,
            this::resetPose,
            () -> getState().Speeds,
            this::driveForPathPlanner,
            new PPHolonomicDriveController(
                Constants.fromPIDValues(Constants.Drivetrain.Auto.translationDynamic),
                Constants.fromPIDValues(Constants.Drivetrain.Auto.thetaDynamic),
                0.02
            ),
            robotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        PathPlannerLogging.setLogActivePathCallback(
            poses -> Logger.recordOutput("Drive/Auto/PathPoses", poses.toArray(Pose2d[]::new))
        );
        PathPlannerLogging.setLogCurrentPoseCallback(
            pose -> Logger.recordOutput("Drive/Auto/CurrentPose", pose)
        );
        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> Logger.recordOutput("Drive/Auto/DesiredPose", pose)
        );

        joystickDrive = new JoystickDrive(this);
    }

    // ── Goal-based architecture ───────────────────────────────────────────────

    /**
     * Set the desired drive goal. Called by the Superstructure (future).
     * Currently commands can also call drive methods directly.
     */
    public void applyGoal(final DriveGoal goal) {
        wantedState = switch (goal) {
            case TELEOP     -> WantedState.TELEOP;
            case LOCK       -> WantedState.LOCK;
            case AUTONOMOUS -> WantedState.AUTONOMOUS;
        };
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case TELEOP     -> SystemState.TELEOP;
            case LOCK       -> SystemState.LOCK;
            case AUTONOMOUS -> SystemState.AUTONOMOUS;
        };
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        systemState = handleStateTransition();

        // Always cache joystick speeds so IntakeGround (and similar) can read them
        joystickSpeeds = joystickDrive.speeds();

        // If JoystickDrive is the active command, apply its speeds
        if (getCurrentCommand() == joystickDrive) {
            driveFieldOriented(joystickSpeeds);
        }

        // Lock wheels if that's the goal and no command has the subsystem
        if (systemState == SystemState.LOCK && getCurrentCommand() == null) {
            setControl(brakeRequest);
        }

        updateVisionFusion();

        // AdvantageKit logging
        final SwerveDriveState state = getState();
        Logger.recordOutput("Drive/Pose",           state.Pose);
        Logger.recordOutput("Drive/Speeds",          state.Speeds);
        Logger.recordOutput("Drive/ModuleStates",    state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets",   state.ModuleTargets);
        Logger.recordOutput("Drive/OdometryPeriod",  state.OdometryPeriod);
    }

    // ── Control methods ───────────────────────────────────────────────────────

    /**
     * Drive field-oriented. vx/vy/omega are in field-relative frame.
     * Called by JoystickDrive (via periodic) and by commands that add corrections.
     */
    public void driveFieldOriented(final ChassisSpeeds fieldRelativeSpeeds) {
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.02);
        setControl(fieldCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    /**
     * Drive robot-oriented. vx/vy/omega are in robot-relative frame.
     * Used by timed auto moves (DriveTime) and rotation-only commands (LookForNote).
     *
     * NOTE: The physical robot has its intake at the "front" end.
     * Positive vx in robot frame drives toward the intake.
     * A 180° inversion is applied to match autonomous path expectations
     * (same as the old Drivetrain.controlRobotOriented behavior).
     */
    public void driveRobotOriented(final ChassisSpeeds robotRelativeSpeeds) {
        // Match old behavior: negate all axes so robot drives in the expected direction
        final ChassisSpeeds inverted = robotRelativeSpeeds.unaryMinus();
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(inverted, 0.02);
        setControl(robotCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    /**
     * Lock wheels in X formation (defensive brake).
     */
    public void halt() {
        setControl(brakeRequest);
    }

    /**
     * PathPlanner output consumer. Called by AutoBuilder during auto path following.
     * Applies inverted robot-oriented speeds to match physical robot orientation.
     */
    public void driveForPathPlanner(final ChassisSpeeds speeds, final DriveFeedforwards ffs) {
        // Scale omega same as old code (0.45 factor), invert all axes
        final ChassisSpeeds adjusted = new ChassisSpeeds(
            -speeds.vxMetersPerSecond,
            -speeds.vyMetersPerSecond,
            -speeds.omegaRadiansPerSecond * 0.45
        );
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(adjusted, 0.02);
        setControl(robotCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    // ── Pose utilities ────────────────────────────────────────────────────────

    /**
     * Returns the current estimated robot pose (always in blue-origin WPILib coordinates).
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Converts robot-relative ChassisSpeeds to field-relative using current heading.
     * Equivalent to the old Drivetrain.rod() method.
     * Used by commands that need to add robot-relative corrections to field-relative joystick speeds.
     */
    public ChassisSpeeds rod(final ChassisSpeeds robotRelativeSpeeds) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getPose().getRotation());
    }

    /**
     * Zero the field-oriented heading (set current heading as "forward") and
     * reset the absolute rotation setpoint in JoystickDrive.
     */
    public void resetAngle() {
        resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
        joystickDrive.forTarget = edu.wpi.first.units.Units.Radians.zero();
    }

    // ── Vision fusion ─────────────────────────────────────────────────────────

    private void updateVisionFusion() {
        // NOTE: LimelightHelpers in this project is v1.2.1 (2023) and does not support
        // MegaTag2 API (SetRobotOrientation / getBotPoseEstimate_wpiBlue_MegaTag2).
        // Upgrading LimelightHelpers to the latest version will enable full MegaTag2 support.
        // For now, use the standard pose estimate from the rear Limelight.

        if (!limelightRear.hasValidTargets()) return;
        if (limelightRear.getNumberOfAprilTags() < 2) return;

        // Reject measurements during fast rotation (> 360 deg/s)
        final double omegaRadPerSec = Math.abs(getState().Speeds.omegaRadiansPerSecond);
        if (omegaRadPerSec > Math.toRadians(360)) return;

        final Pose2d visionPose = limelightRear.getPose2d();

        // Sanity check: reject if vision pose is too far from odometry estimate
        final double poseDiff = getPose().getTranslation().getDistance(visionPose.getTranslation());
        if (poseDiff >= 0.5) return;

        // Use FPGA timestamp without manual latency offset
        // (the 0.3s offset from the old code has been removed per calibration)
        addVisionMeasurement(visionPose, Timer.getFPGATimestamp());

        Logger.recordOutput("Drive/VisionPose", visionPose);
    }
}
