package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.subsystems.drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.superstructure.SuperstructureContext;
import frc.robot.vision.Limelight;

/**
 * Swerve drivetrain subsystem.
 *
 * Owns all drive behavior via the goal-based pattern:
 *   applyGoal(SuperstructureContext) → wantedState → periodic() → hardware
 *
 * Extends CTRE's generated TunerSwerveDrivetrain directly (no intermediate class).
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // ── Goal / state machine ──────────────────────────────────────────────────

    enum WantedState { TELEOP, LOCK, AIM_SPEAKER, TRACK_NOTE }
    enum SystemState  { TELEOP, LOCK, AIM_SPEAKER, TRACK_NOTE }

    private WantedState wantedState = WantedState.TELEOP;
    private SystemState systemState = SystemState.TELEOP;

    // ── Joystick suppliers (injected by RobotContainer after OI is built) ────

    private Supplier<Double> axialSupplier    = () -> 0.0;
    private Supplier<Double> lateralSupplier  = () -> 0.0;
    private Supplier<Double> rotationSupplier = () -> 0.0;

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

    // Feedforward for limelight-based rotational correction during AIM_SPEAKER.
    private static final SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0, 5);

    // ── Operator perspective ─────────────────────────────────────────────────

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    // ── Simulation ───────────────────────────────────────────────────────────

    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // ── SysId characterization ───────────────────────────────────────────────

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)), null, this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(7), null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null, this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // ── Limelights ───────────────────────────────────────────────────────────

    public final Limelight limelightNote    = new Limelight("limelight-note");
    public final Limelight limelightShooter = new Limelight("limelight-shooter");
    public final Limelight limelightRear    = new Limelight("limelight-rear");

    /**
     * Cached joystick chassis speeds, updated each cycle during TELEOP.
     * Read by IntakeGround to apply vision-based corrections on top of driver input.
     */
    public ChassisSpeeds joystickSpeeds = new ChassisSpeeds();

    // ── Constructor ──────────────────────────────────────────────────────────

    public CommandSwerveDrivetrain() {
        super(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }

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
            final Supplier<Double> rotation) {
        this.axialSupplier    = axial;
        this.lateralSupplier  = lateral;
        this.rotationSupplier = rotation;
    }

    /**
     * Set the desired drive behavior for this cycle.
     * Called every 20 ms by {@link frc.robot.superstructure.Superstructure#periodic()}.
     */
    public void applyGoal(final SuperstructureContext ctx) {
        wantedState = switch (ctx.goal().drive()) {
            case TELEOP      -> WantedState.TELEOP;
            case LOCK        -> WantedState.LOCK;
            case AUTONOMOUS  -> WantedState.TELEOP;
            case AIM_SPEAKER -> WantedState.AIM_SPEAKER;
            case TRACK_NOTE  -> WantedState.TRACK_NOTE;
        };
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Apply operator perspective for alliance-correct field-centric driving
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        systemState = handleStateTransition();
        applyState();

        final SwerveDriveState state = getState();
        Logger.recordOutput("Drive/Pose",          state.Pose);
        Logger.recordOutput("Drive/Speeds",        state.Speeds);
        Logger.recordOutput("Drive/ModuleStates",  state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
        Logger.recordOutput("Drive/SystemState",   systemState.toString());
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
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

    // ── AIM_SPEAKER ──────────────────────────────────────────────────────────

    private void applyAimSpeaker() {
        final boolean facingForward = getPose().getRotation().getCos() < 0;
        final var t = translation();

        final double rotCorrection;
        if (facingForward && limelightShooter.hasValidTargets()) {
            final double verticalOffset = limelightShooter.getTargetVerticalOffset().in(Units.Rotations);
            rotCorrection = MathUtil.clamp(aimFF.calculate(verticalOffset), -0.125, 0.125);
        } else if (!facingForward && limelightRear.hasValidTargets()) {
            final double horizontalOffset = limelightRear.getTargetHorizontalOffset().in(Units.Rotations);
            rotCorrection = aimFF.calculate(horizontalOffset);
        } else {
            rotCorrection = -MathUtil.applyDeadband(rotationSupplier.get(), 0.075);
        }

        final ChassisSpeeds s = new ChassisSpeeds(
            t.getX(),
            t.getY(),
            Constants.Drivetrain.maxAngularVelocity.times(rotCorrection).in(Units.RadiansPerSecond)
        );
        joystickSpeeds = s;
        driveFieldOriented(s);
    }

    // ── TRACK_NOTE ───────────────────────────────────────────────────────────

    private void applyTrackNote() {
        final ChassisSpeeds joystick = computeJoystickSpeeds();
        joystickSpeeds = joystick;

        if (!limelightNote.hasValidTargets()) {
            driveFieldOriented(joystick);
            return;
        }

        final double horizontalOffsetDeg = limelightNote.getTargetHorizontalOffset().in(Units.Degrees);
        final double horizontalOffsetRot = limelightNote.getTargetHorizontalOffset().in(Units.Rotations);

        final ChassisSpeeds correction = robotToField(new ChassisSpeeds(
            -10.0 / (Math.abs(horizontalOffsetDeg) + 1),
            horizontalOffsetRot * 10,
            0
        ));

        Logger.recordOutput("Drive/TrackNote/CorrectionX", correction.vxMetersPerSecond);
        Logger.recordOutput("Drive/TrackNote/CorrectionY", correction.vyMetersPerSecond);
        driveFieldOriented(joystick.plus(correction));
    }

    // ── Joystick computation ─────────────────────────────────────────────────

    private ChassisSpeeds computeJoystickSpeeds() {
        final Translation2d t = translation();
        final double rotation = -MathUtil.applyDeadband(rotationSupplier.get(), 0.1);
        final double omega = Constants.Drivetrain.maxAngularVelocity
            .times(rotation).in(Units.RadiansPerSecond);
        return new ChassisSpeeds(t.getX(), t.getY(), omega);
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

    // ── Control methods ──────────────────────────────────────────────────────

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
    public ChassisSpeeds robotToField(final ChassisSpeeds robotRelativeSpeeds) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getPose().getRotation());
    }

    /** Zeros the field-oriented heading to "away from driver station" for current alliance. */
    public void resetAngle() {
        seedFieldCentric();
    }

    // ── SysId commands ───────────────────────────────────────────────────────

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // ── Vision measurement overrides (FPGA timestamp correction) ─────────────

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs
        );
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // ── Simulation ───────────────────────────────────────────────────────────

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
