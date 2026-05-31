package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.climber.ClimberGoal;
import frc.robot.superstructure.SuperstructureContext;

public class Climber extends SubsystemBase {

    enum WantedState { IDLE, INITIALIZE, DEPLOY, RETRACT, LOCKED }
    enum SystemState { IDLE, INITIALIZING_RAISE, INITIALIZING_HOME, DEPLOYING, RETRACTING, LOCKED }

    public final ClimberIO io;
    public final ClimberIOInputs inputs = new ClimberIOInputs();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;
    private WantedState prevWantedState = WantedState.IDLE;
    private double initializeStartPosition;

    public Climber() {
        this.io = switch (Constants.mode) {
            case REAL -> new ClimberIOReal();
            default   -> new ClimberIO() {};
        };
    }

    public void applyGoal(final SuperstructureContext ctx) {
        ClimberGoal desired = ctx.goal().climber();
        // Interlock: block DEPLOY if shooter pivot is not at safe angle
        if (desired == ClimberGoal.DEPLOY && !ctx.shooterState().atSafeAngle()) {
            wantedState = WantedState.IDLE;
            return;
        }
        wantedState = switch (desired) {
            case IDLE       -> WantedState.IDLE;
            case INITIALIZE -> WantedState.INITIALIZE;
            case DEPLOY     -> WantedState.DEPLOY;
            case RETRACT    -> WantedState.RETRACT;
            case LOCKED     -> WantedState.LOCKED;
        };
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Climber", inputs); // enable after ./gradlew build generates ClimberIOInputsAutoLogged
        systemState = handleStateTransition();
        applyState();
        Logger.recordOutput("Climber/WantedState", wantedState.toString());
        Logger.recordOutput("Climber/SystemState",  systemState.toString());
    }

    private SystemState handleStateTransition() {
        if (wantedState == WantedState.INITIALIZE && prevWantedState != WantedState.INITIALIZE) {
            initializeStartPosition = inputs.position;
        }
        prevWantedState = wantedState;

        return switch (wantedState) {
            case IDLE    -> SystemState.IDLE;
            case DEPLOY  -> SystemState.DEPLOYING;
            case RETRACT -> SystemState.RETRACTING;
            case LOCKED  -> SystemState.LOCKED;
            case INITIALIZE -> {
                if (inputs.home) {
                    yield SystemState.IDLE;
                } else if (inputs.position > initializeStartPosition + Constants.Climber.initializeRaiseDistance) {
                    yield SystemState.INITIALIZING_HOME;
                } else {
                    yield SystemState.INITIALIZING_RAISE;
                }
            }
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLE               -> io.set(inputs.position);
            case INITIALIZING_RAISE -> io.override(0.25);
            case INITIALIZING_HOME  -> io.override(-0.75);
            case DEPLOYING          -> io.set(Constants.Climber.max);
            case RETRACTING         -> io.set(0);
            case LOCKED             -> io.override(0);
        }
    }

    public BaseStatusSignal[] getStatusSignals() {
        return io.getStatusSignals();
    }

    public record ClimberState(double position, boolean isAtHome, boolean isDeployed, boolean isRetracted) {}

    public ClimberState getState() {
        return new ClimberState(
            inputs.position,
            inputs.home,
            inputs.position >= Constants.Climber.max - 1.0,
            inputs.position <= 1.0
        );
    }
}
