package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.superstructure.SuperstructureContext;

/**
 * Shooter + intake combined subsystem.
 *
 * <p>Intake is mechanically part of the shooter and never operates independently.
 * All intake behavior is expressed through {@link ShooterGoal#INTAKE}.
 *
 * <p>Hardware:
 * <ul>
 *   <li>TalonFX: pivot motor (CANcoder absolute feedback)</li>
 *   <li>TalonFX x2: flywheels A and B</li>
 *   <li>TalonSRX: feeder roller</li>
 *   <li>TalonSRX: intake roller</li>
 *   <li>Servo x2: amp bar</li>
 * </ul>
 */
public class Shooter extends SubsystemBase {

    // ── Goal / state machine ──────────────────────────────────────────────────

    public enum WantedState { HOME, INTAKE, SHOOT_SPEAKER, SHOOT_FIXED, AMP, FERRY }

    public enum SystemState {
        HOME,
        INTAKE,
        SHOOT_SPEAKER_AIMING,
        SHOOT_SPEAKER_FIRING,
        SHOOT_FIXED,
        AMP,
        FERRY
    }

    private WantedState wantedState     = WantedState.HOME;
    private SystemState systemState     = SystemState.HOME;
    private WantedState prevWantedState = WantedState.HOME;

    // ── IO / inputs ───────────────────────────────────────────────────────────

    public final ShooterIO io;
    public final ShooterIOInputs inputs = new ShooterIOInputs();

    // ── Internal sequencing state ─────────────────────────────────────────────

    /** FPGA timestamp when feeder first fired in the current goal, or -1 if not yet. */
    private double firedTime = -1;

    // ── Auto-aim controllers ──────────────────────────────────────────────────

    private final PIDController pitchPID =
        new PIDController(0.75, 0, 0);

    // ── Constructor ───────────────────────────────────────────────────────────

    public Shooter() {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default   -> new ShooterIO() {};
        };
        SmartDashboard.putData("Shooter/PitchPID", pitchPID);
    }

    // ── Goal API ──────────────────────────────────────────────────────────────

    /**
     * Called every 20 ms by {@link frc.robot.superstructure.Superstructure#periodic()}.
     * Sets {@code wantedState} from the goal; does not write to hardware.
     */
    public void applyGoal(final SuperstructureContext ctx) {
        wantedState = switch (ctx.goal().shooter()) {
            case HOME          -> WantedState.HOME;
            case INTAKE        -> WantedState.INTAKE;
            case SHOOT_SPEAKER -> WantedState.SHOOT_SPEAKER;
            case SHOOT_FIXED   -> WantedState.SHOOT_FIXED;
            case AMP           -> WantedState.AMP;
            case FERRY         -> WantedState.FERRY;
        };
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Shooter", inputs); // enable after ./gradlew build generates ShooterIOInputsAutoLogged

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Shooter/WantedState", wantedState.toString());
        Logger.recordOutput("Shooter/SystemState",  systemState.toString());
    }

    // ── State machine ─────────────────────────────────────────────────────────

    private SystemState handleStateTransition() {
        if (wantedState != prevWantedState) {
            firedTime       = -1;
            prevWantedState = wantedState;
        }

        return switch (wantedState) {
            case HOME          -> SystemState.HOME;
            case INTAKE        -> SystemState.INTAKE;
            case SHOOT_SPEAKER -> firedTime != -1
                                    ? SystemState.SHOOT_SPEAKER_FIRING
                                    : SystemState.SHOOT_SPEAKER_AIMING;
            case SHOOT_FIXED   -> SystemState.SHOOT_FIXED;
            case AMP           -> SystemState.AMP;
            case FERRY         -> SystemState.FERRY;
        };
    }

    private void applyState() {
        switch (systemState) {
            case HOME                                    -> applyHome();
            case INTAKE                                  -> applyIntake();
            case SHOOT_SPEAKER_AIMING, SHOOT_SPEAKER_FIRING -> applyShootSpeaker();
            case SHOOT_FIXED                             -> applyShootFixed();
            case AMP                                     -> applyAmp();
            case FERRY                                   -> applyFerry();
        }
    }

    // ── Per-state hardware routines ───────────────────────────────────────────

    private void applyHome() {
        io.retractAmpBar();
        io.rotate(inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.idleEmpty);
        io.runFlywheels(0);
        io.runFeeder(Demand.Halt);
        io.runIntake(Demand.Halt);
    }

    private void applyIntake() {
        final boolean pivotReady = Math.abs(
            inputs.angle.in(Units.Degrees) - Constants.Shooter.intakeGround.in(Units.Degrees)
        ) <= 1.5;

        io.rotate(Constants.Shooter.intakeGround);
        io.runFlywheels(-0.25);
        io.runFeeder(Demand.Reverse);
        io.runIntake(pivotReady ? Demand.Forward : Demand.Halt);
    }

    private void applyShootSpeaker() {
        // cosine < 0 → robot is facing away from the driver station (toward the speaker)
        final boolean facingForward    = Robot.cont.drivetrain.getPose().getRotation().getCos() < 0;
        final boolean isShooterForward = inputs.angle.in(Units.Degrees) - 90 < 0;

        io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
        Robot.cont.drivetrain.limelightShooter.setPipeline(facingForward ? 0 : 1);

        final boolean flywheelAtSpeed =
            inputs.flywheelSpeedA.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();
        final boolean pivotVelocityOk =
            Math.abs(inputs.angleSpeed.in(Units.RotationsPerSecond))
                < Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);

        if (facingForward && isShooterForward) {
            if (Robot.cont.drivetrain.limelightShooter.hasValidTargets()) {
                // limelight is mounted sideways: horizontal offset → pitch, vertical offset → yaw
                final var pitchOffset = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
                final var yawOffset = Robot.cont.drivetrain.limelightShooter
                    .getTargetVerticalOffset()
                    .times(isShooterForward ? 1 : -1);

                final boolean pivotInPosition =
                    Math.abs(pitchOffset.in(Units.Degrees)) < Tuning.shootSpeakerPivotThreshold.get();

                Logger.recordOutput("Shooter/ShootSpeaker/tx", pitchOffset.in(Units.Degrees));
                Logger.recordOutput("Shooter/ShootSpeaker/ty", yawOffset.in(Units.Degrees));
                Logger.recordOutput("Shooter/ShootSpeaker/PivotInPosition", pivotInPosition);
                Logger.recordOutput("Shooter/ShootSpeaker/FlywheelAtSpeed", flywheelAtSpeed);
                Logger.recordOutput("Shooter/ShootSpeaker/PivotVelocityOk", pivotVelocityOk);

                if (!pivotInPosition && firedTime == -1) {
                    io.rotate(
                        Units.Rotations.of(
                            inputs.angle.in(Units.Rotations)
                                + pitchPID.calculate(pow(pitchOffset.in(Units.Rotations), Tuning.shootSpeakerExponent.get()))
                        )
                    );
                } else if ((flywheelAtSpeed && pivotVelocityOk && yawOffset.in(Units.Degrees) < 10) || firedTime != -1) {
                    io.runFeeder(Demand.Forward);
                    if (firedTime == -1) firedTime = Timer.getFPGATimestamp();
                }

            } else {
                io.rotate(Constants.Shooter.readyShootFront);
            }

        } else if (!facingForward && Robot.cont.drivetrain.limelightRear.hasValidTargets()) {
            io.rotate(Constants.Shooter.readyShootRear);
            if ((flywheelAtSpeed && pivotVelocityOk) || firedTime != -1) {
                io.runFeeder(Demand.Forward);
                if (firedTime == -1) firedTime = Timer.getFPGATimestamp();
            }

        } else {
            io.rotate(facingForward ? Constants.Shooter.readyShootFront : Constants.Shooter.readyShootRear);
        }

        Logger.recordOutput("Shooter/ShootSpeaker/FiredTime", firedTime != -1);
    }

    private void applyShootFixed() {
        final var angle = Units.Degrees.of(Tuning.subAngle.get());
        io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
        io.rotate(angle);

        final boolean pivotAngle  =
            Math.abs(inputs.angle.in(Units.Degrees) - angle.in(Units.Degrees)) < 1.25;
        final boolean flywheelSpeed =
            inputs.flywheelSpeedA.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();
        final boolean pivotVelocity =
            Math.abs(inputs.angleSpeed.in(Units.RotationsPerSecond))
                < Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);

        Logger.recordOutput("Shooter/ShootFixed/PivotAngle",    pivotAngle);
        Logger.recordOutput("Shooter/ShootFixed/FlywheelSpeed",  flywheelSpeed);
        Logger.recordOutput("Shooter/ShootFixed/PivotVelocity",  pivotVelocity);
        Logger.recordOutput("Shooter/ShootFixed/Fired",          firedTime != -1);

        if ((pivotAngle && flywheelSpeed && pivotVelocity) || firedTime != -1) {
            io.runFeeder(Demand.Forward);
            if (firedTime == -1) firedTime = Timer.getFPGATimestamp();
        }
    }

    private void applyAmp() {
        io.extendAmpBar();
        io.rotate(Units.Degrees.of(Tuning.ampAngle.get()));
        io.runFlywheels(Tuning.ampPower.get());

        final boolean angleReady =
            Math.abs(inputs.angle.in(Units.Degrees) - Tuning.ampAngle.get()) <= 20.5;

        if (angleReady || firedTime != -1) {
            io.runFeeder(Demand.Forward);
            if (firedTime == -1) firedTime = Timer.getFPGATimestamp();
        }
    }

    private void applyFerry() {
        final var angle = Units.Degrees.of(Tuning.ferryAngle.get());
        io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
        io.rotate(angle);

        final boolean pivotAngle   =
            Math.abs(inputs.angle.in(Units.Degrees) - angle.in(Units.Degrees)) < 1.25;
        final boolean flywheelSpeed =
            inputs.flywheelSpeedA.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();

        if ((pivotAngle && flywheelSpeed) || firedTime != -1) {
            io.runFeeder(Demand.Forward);
            if (firedTime == -1) firedTime = Timer.getFPGATimestamp();
        }
    }

    // ── State snapshot (used by SuperstructureContext and auto routines) ───────

    /**
     * Read-only snapshot of shooter state for {@link frc.robot.superstructure.SuperstructureContext}.
     *
     * @param holdingNote  true when beam-break sensor detects a note
     * @param shotComplete true when feeder has been running for at least {@code fireTimeout}
     * @param atSafeAngle  true when pivot is low enough for climber deployment
     */
    public record ShooterState(boolean holdingNote, boolean shotComplete, boolean atSafeAngle) {}

    public ShooterState getState() {
        final boolean shotComplete =
            firedTime != -1 && Timer.getFPGATimestamp() - firedTime >= Constants.Shooter.fireTimeout;
        final boolean atSafeAngle =
            inputs.angle.in(Units.Degrees) < 20.0;
        return new ShooterState(inputs.holdingNote, shotComplete, atSafeAngle);
    }

    // ── Helper ────────────────────────────────────────────────────────────────

    private double pow(final double base, final double exp) {
        return Math.copySign(Math.pow(Math.abs(base), exp), base);
    }
}
