package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.Limelight;

/**
 * Swerve drivetrain subsystem backed by CTRE's CommandSwerveDrivetrain.
 *
 * Stripped to the minimum needed to confirm basic driving works.
 * PathPlanner, vision fusion, and goal-based architecture will be
 * added incrementally once driving is verified.
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {

    // ── Swerve requests (allocate once, mutate via with-methods) ─────────────
    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    // ── Vision standard deviations ───────────────────────────────────────────
    private static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.7, 0.7, 9999999);

    // ── Limelights (used by shooter/intake commands) ─────────────────────────
    public final Limelight limelightNote    = new Limelight("limelight-note");
    public final Limelight limelightShooter = new Limelight("limelight-shooter");
    public final Limelight limelightRear    = new Limelight("limelight-rear");

    /**
     * Cached joystick chassis speeds, updated by JoystickDrive.execute().
     * Read by IntakeGround to add vision-based corrections.
     */
    public ChassisSpeeds joystickSpeeds = new ChassisSpeeds();

    // ── Constructor ──────────────────────────────────────────────────────────
    public DriveSubsystem() {
        super(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        CommandScheduler.getInstance().registerSubsystem(this);
        setVisionMeasurementStdDevs(VISION_STD_DEVS);
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        super.periodic(); // applies alliance-relative operator perspective

        final SwerveDriveState state = getState();
        Logger.recordOutput("Drive/Pose",          state.Pose);
        Logger.recordOutput("Drive/Speeds",         state.Speeds);
        Logger.recordOutput("Drive/ModuleStates",   state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets",  state.ModuleTargets);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
    }

    // ── Control methods ───────────────────────────────────────────────────────

    public void driveFieldOriented(final ChassisSpeeds fieldRelativeSpeeds) {
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.02);
        setControl(fieldCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    public void driveRobotOriented(final ChassisSpeeds robotRelativeSpeeds) {
        final ChassisSpeeds inverted = robotRelativeSpeeds.unaryMinus();
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(inverted, 0.02);
        setControl(robotCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    public void halt() {
        setControl(brakeRequest);
    }

    // ── Pose utilities ────────────────────────────────────────────────────────

    public Pose2d getPose() {
        return getState().Pose;
    }

    /** Converts robot-relative speeds to field-relative using current heading. */
    public ChassisSpeeds rod(final ChassisSpeeds robotRelativeSpeeds) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getPose().getRotation());
    }

    /** Zeros the field-oriented heading (sets current heading as forward). */
    public void resetAngle() {
        resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
    }
}
