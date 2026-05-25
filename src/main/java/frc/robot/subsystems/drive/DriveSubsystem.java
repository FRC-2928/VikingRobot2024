package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.SuperstructureContext;
import frc.robot.vision.Limelight;

/**
 * Swerve drivetrain subsystem backed by CTRE's CommandSwerveDrivetrain.
 *
 * Owns all drive behavior via the goal-based pattern:
 *   applyGoal(SuperstructureContext) → wantedState → periodic() → hardware
 *
 * Callers set a goal each cycle via Commands.run(); the subsystem decides how to
 * fulfill it. The default goal is TELEOP (driver joystick). LOCK engages X-formation
 * braking. During autonomous, TELEOP automatically becomes LOCK so the robot holds
 * position when no auto command owns the drivetrain.
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {

    // ── Goal / state machine ──────────────────────────────────────────────────

    enum WantedState { TELEOP, LOCK, AIM_SPEAKER, TRACK_NOTE }
    enum SystemState  { TELEOP, LOCK, AIM_SPEAKER, TRACK_NOTE }

    private WantedState wantedState = WantedState.TELEOP;
    private SystemState systemState = SystemState.TELEOP;

    // ── Joystick suppliers (injected by RobotContainer after OI is built) ────

    private Supplier<Double> axialSupplier    = () -> 0.0;
    private Supplier<Double> lateralSupplier  = () -> 0.0;
    private Supplier<Double> forXSupplier     = () -> 0.0;
    private Supplier<Double> forYSupplier     = () -> 0.0;
    private Supplier<String> driveModeSupplier = () -> "Swerve Drive";

    // ── Absolute-rotation PID (used in "Field Oriented" drive mode) ───────────

    private final ProfiledPIDController absoluteController =
        Constants.Drivetrain.absoluteRotationPID
            .createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);

    private Angle  forTarget    = Units.Radians.zero();
    private double forMagnitude = 0.5;

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

    // Feedforward used for limelight-based rotational correction during AIM_SPEAKER.
    // kV=5 matches the original ShootSpeaker auto-align scaling; output is clamped to ±0.125 rad/s.
    private static final SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0, 5);

    // ── Limelights (used by shooter/intake commands) ─────────────────────────

    public final Limelight limelightNote    = new Limelight("limelight-note");
    public final Limelight limelightShooter = new Limelight("limelight-shooter");
    public final Limelight limelightRear    = new Limelight("limelight-rear");

    /**
     * Cached joystick chassis speeds, updated each cycle during TELEOP.
     * Read by IntakeGround to apply vision-based corrections on top of driver input.
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

        absoluteController.enableContinuousInput(-0.5, 0.5);

        CommandScheduler.getInstance().registerSubsystem(this);
        setVisionMeasurementStdDevs(VISION_STD_DEVS);
    }

    // ── Goal API ──────────────────────────────────────────────────────────────

    /**
     * Inject joystick suppliers from OI. Call once from RobotContainer after
     * both the drivetrain and OI objects are constructed.
     */
    public void configureJoystick(
            final Supplier<Double> axial,
            final Supplier<Double> lateral,
            final Supplier<Double> forX,
            final Supplier<Double> forY,
            final Supplier<String> driveMode) {
        this.axialSupplier    = axial;
        this.lateralSupplier  = lateral;
        this.forXSupplier     = forX;
        this.forYSupplier     = forY;
        this.driveModeSupplier = driveMode;
    }

    /**
     * Set the desired drive behavior for this cycle.
     * Called every 20 ms by {@link frc.robot.superstructure.Superstructure#periodic()}.
     */
    public void applyGoal(final SuperstructureContext ctx) {
        wantedState = switch (ctx.goal().drive()) {
            case TELEOP      -> WantedState.TELEOP;
            case LOCK        -> WantedState.LOCK;
            case AUTONOMOUS  -> WantedState.TELEOP; // auto commands drive via driveFieldOriented() directly
            case AIM_SPEAKER -> WantedState.AIM_SPEAKER;
            case TRACK_NOTE  -> WantedState.TRACK_NOTE;
        };
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        super.periodic(); // applies alliance-relative operator perspective + odometry

        systemState = handleStateTransition();
        applyState();

        final SwerveDriveState state = getState();
        Logger.recordOutput("Drive/Pose",          state.Pose);
        Logger.recordOutput("Drive/Speeds",         state.Speeds);
        Logger.recordOutput("Drive/ModuleStates",   state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets",  state.ModuleTargets);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
        Logger.recordOutput("Drive/SystemState",    systemState.toString());
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            // During autonomous, fall back to X-lock when no auto command owns the drivetrain.
            case TELEOP      -> DriverStation.isAutonomous() ? SystemState.LOCK : SystemState.TELEOP;
            case LOCK        -> SystemState.LOCK;
            case AIM_SPEAKER -> DriverStation.isAutonomous() ? SystemState.LOCK : SystemState.AIM_SPEAKER;
            case TRACK_NOTE  -> DriverStation.isAutonomous() ? SystemState.LOCK : SystemState.TRACK_NOTE;
        };
    }

    private void applyState() {
        switch (systemState) {
            case TELEOP -> {
                final ChassisSpeeds s = computeJoystickSpeeds();
                joystickSpeeds = s;
                driveFieldOriented(s);
            }
            case LOCK        -> halt();
            case AIM_SPEAKER -> applyAimSpeaker();
            case TRACK_NOTE  -> applyTrackNote();
        }
    }

    /**
     * Joystick translation with limelight-based rotational correction for speaker shots.
     * Replicates the auto-align logic previously in ShootSpeaker.execute().
     */
    private void applyAimSpeaker() {
        final boolean facingForward = getPose().getRotation().getCos() < 0;
        final var t = translation();

        final double rotCorrection;
        if (facingForward && limelightShooter.hasValidTargets()) {
            // limelight mounted sideways: vertical offset is the yaw error
            final double yo = limelightShooter.getTargetVerticalOffset().in(Units.Rotations);
            rotCorrection = MathUtil.clamp(aimFF.calculate(yo), -0.125, 0.125);
        } else if (!facingForward && limelightRear.hasValidTargets()) {
            final double ho = limelightRear.getTargetHorizontalOffset().in(Units.Rotations);
            rotCorrection = aimFF.calculate(ho);
        } else {
            // No limelight target — fall back to joystick rotation
            rotCorrection = -MathUtil.applyDeadband(forXSupplier.get(), 0.075);
        }

        final ChassisSpeeds s = new ChassisSpeeds(
            t.getX(),
            t.getY(),
            Constants.Drivetrain.maxAngularVelocity.times(rotCorrection).in(Units.RadiansPerSecond)
        );
        joystickSpeeds = s;
        driveFieldOriented(s);
    }

    /**
     * Joystick translation with additive limelight-based correction toward the note.
     * Driver retains full control — the correction is added on top of joystick input.
     * Falls back to pure joystick when limelight has no valid target.
     */
    private void applyTrackNote() {
        final ChassisSpeeds joystick = computeJoystickSpeeds();
        joystickSpeeds = joystick;

        if (!limelightNote.hasValidTargets()) {
            driveFieldOriented(joystick);
            return;
        }

        final double horizontalOffsetDeg = limelightNote.getTargetHorizontalOffset().in(Units.Degrees);
        final double horizontalOffsetRot = limelightNote.getTargetHorizontalOffset().in(Units.Rotations);

        // Robot-relative: X drives toward note, Y steers laterally to center it.
        final ChassisSpeeds correction = rod(new ChassisSpeeds(
            -10.0 / (Math.abs(horizontalOffsetDeg) + 1),
            horizontalOffsetRot * 10,
            0
        ));

        Logger.recordOutput("Drive/TrackNote/CorrectionX", correction.vxMetersPerSecond);
        Logger.recordOutput("Drive/TrackNote/CorrectionY", correction.vyMetersPerSecond);
        driveFieldOriented(joystick.plus(correction));
    }

    // ── Joystick computation (was JoystickDrive) ─────────────────────────────

    private ChassisSpeeds computeJoystickSpeeds() {
        final Translation2d t = translation();
        return new ChassisSpeeds(
            t.getX(),
            t.getY(),
            theta().in(Units.RadiansPerSecond)
        );
    }

    private Translation2d translation() {
        final double axial   = MathUtil.applyDeadband(axialSupplier.get(),   0.1);
        final double lateral = MathUtil.applyDeadband(lateralSupplier.get(), 0.1);

        final Rotation2d direction = Rotation2d.fromRadians(Math.atan2(lateral, axial));
        final double magnitude = Math.pow(MathUtil.clamp(Math.hypot(axial, lateral), 0, 1), 2);

        final double dx = Math.cos(direction.getRadians()) * magnitude;
        final double dy = Math.sin(direction.getRadians()) * magnitude;

        final LinearVelocity vx = Constants.Drivetrain.maxVelocity.times(dx);
        final LinearVelocity vy = Constants.Drivetrain.maxVelocity.times(dy);

        return new Translation2d(vx.in(Units.MetersPerSecond), vy.in(Units.MetersPerSecond));
    }

    private AngularVelocity theta() {
        final double theta;

        if ("Swerve Drive".equals(driveModeSupplier.get())) {
            theta = -MathUtil.applyDeadband(forXSupplier.get(), 0.075);
        } else {
            // Field Oriented: right stick angle sets target heading, PID drives toward it.
            final double rotX = forXSupplier.get();
            final double rotY = forYSupplier.get();

            forMagnitude = Math.hypot(rotX, rotY);
            Logger.recordOutput("Drive/AbsoluteRotation/Magnitude", forMagnitude);

            if (forMagnitude > 0.5) forTarget = Units.Radians.of(-Math.atan2(rotX, rotY));
            Logger.recordOutput("Drive/AbsoluteRotation/Target", forTarget);

            forMagnitude = forMagnitude * 0.5 + 0.5;

            final double measurement = getPose().getRotation().getRotations();
            final double setpoint    = forTarget.in(Units.Rotations);

            theta = MathUtil.applyDeadband(
                -(absoluteController.calculate(measurement, setpoint)),
                0.075
            );
        }

        return Constants.Drivetrain.maxAngularVelocity.times(theta * forMagnitude);
    }

    // ── Control methods (used directly by auto/vision commands) ──────────────

    public void driveFieldOriented(final ChassisSpeeds fieldRelativeSpeeds) {
        final ChassisSpeeds discretized = ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.02);
        setControl(fieldCentricRequest
            .withVelocityX(discretized.vxMetersPerSecond)
            .withVelocityY(discretized.vyMetersPerSecond)
            .withRotationalRate(discretized.omegaRadiansPerSecond));
    }

    public void driveRobotOriented(final ChassisSpeeds robotRelativeSpeeds) {
        final ChassisSpeeds inverted    = robotRelativeSpeeds.unaryMinus();
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
