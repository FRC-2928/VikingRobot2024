package frc.robot.utils;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.stream.Stream;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Pairs a group of CAN status signals with the logic that writes their values
 * into an inputs struct. This enables "declare once, use everywhere":
 *
 * <ul>
 *   <li><b>Registration:</b> {@link #signals()} provides the array for
 *       {@code Superstructure.registerSignals()}</li>
 *   <li><b>Update:</b> {@link #update(Object)} copies current signal values
 *       into the inputs struct each cycle</li>
 * </ul>
 *
 * <p>Usage per device group:
 * <pre>
 * SignalBundle&lt;ShooterIOInputs&gt; hoodSignals = new SignalBundle&lt;&gt;(
 *     new BaseStatusSignal[] { hoodAngle, hoodVelocity, hoodCurrent },
 *     inputs -> {
 *         inputs.hoodAngle = hoodAngle.getValue();
 *         inputs.hoodVelocity = hoodVelocity.getValue();
 *         inputs.hoodCurrent = hoodCurrent.getValue();
 *     }
 * );
 * </pre>
 *
 * <p>For repeated identical structure (e.g., 4 swerve modules), use a factory
 * method that returns a {@code SignalBundle} per instance.
 *
 * @param <T> the inputs struct type this bundle writes to
 */
public class SignalBundle<T> {
    private final BaseStatusSignal[] mSignals;
    private final Consumer<T> mUpdater;

    public SignalBundle(BaseStatusSignal[] signals, Consumer<T> updater) {
        this.mSignals = signals;
        this.mUpdater = updater;
    }

    public BaseStatusSignal[] signals() {
        return mSignals;
    }

    public void update(T inputs) {
        mUpdater.accept(inputs);
    }

    /**
     * Collects all signals from multiple bundles into a single flat array.
     * Useful for passing to {@code Superstructure.registerSignals()}.
     */
    @SafeVarargs
    public static <T> BaseStatusSignal[] collectSignals(SignalBundle<T>... bundles) {
        return Arrays.stream(bundles)
            .flatMap(b -> Stream.of(b.signals()))
            .toArray(BaseStatusSignal[]::new);
    }
}
