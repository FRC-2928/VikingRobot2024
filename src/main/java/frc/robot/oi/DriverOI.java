package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.moveAxial = () -> -this.controller.getLeftY();
		this.moveLateral = this.controller::getLeftX;
		
		if (Constants.currentMode == Mode.REAL) {
			this.moveTheta = this.controller::getRightX;
			this.moveRotationX = this.controller::getRightX;
			this.moveRotationY = () -> -this.controller.getRightY();
		} else {
			this.moveTheta = () -> this.hid.getRawAxis(2);
			this.moveRotationX = () -> this.hid.getRawAxis(2);
			this.moveRotationY = () -> this.hid.getRawAxis(3);
		}
		
		this.slow = this.controller::getLeftTriggerAxis;

        this.shoot = new Trigger(() -> this.controller.getRightTriggerAxis() > 0.5);
		this.alignShooter = this.controller.rightBumper();

		this.lock = this.controller.leftBumper();

		this.resetFOD = this.controller.y();
	}

	public final Supplier<Double> moveAxial;
	public final Supplier<Double> moveLateral;

	public final Supplier<Double> moveTheta;

	public final Supplier<Double> moveRotationX;
	public final Supplier<Double> moveRotationY;

	public final Supplier<Double> slow;

    public final Trigger shoot;

	public final Trigger alignShooter;

	public final Trigger lock;

	public final Trigger resetFOD;
}
