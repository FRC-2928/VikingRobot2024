package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO;
import frc.robot.subsystems.climber.ClimberGoal;
import frc.robot.subsystems.shooter.ShooterGoal;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climberDown = this.controller.x();
		this.climberUp = this.controller.y();

		this.climberOverrideLower = this.controller.povDown();
		this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();

		this.intakeOut = this.controller.b();
		this.intakeIn = this.controller.a();

		this.fixedShoot = this.controller.leftTrigger();
		this.overrideShoot = this.controller.rightTrigger();

		this.foc = this.controller.rightBumper();
	}

	public final Trigger climberDown;
	public final Trigger climberUp;

	public final Trigger climberOverrideLower;
	public final Trigger climberOverrideRaise;

	public final Trigger initializeClimber;

	public final Trigger intakeOut;
	public final Trigger intakeIn;
	public final Trigger fixedShoot;

	public final Trigger overrideShoot;

	public final Trigger foc;

	public void configureControls() {
		this.climberDown
			.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.RETRACT)))
			.onFalse(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.IDLE)));
		this.climberUp
			.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
				Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.DEPLOY);
				Robot.cont.superstructure.resolver.setShooterIntent(ShooterGoal.HOME);
			}))
			.onFalse(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
				Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.IDLE);
				Robot.cont.superstructure.resolver.setShooterIntent(ShooterGoal.HOME);
			}));

		this.climberOverrideLower.whileTrue(new FunctionalCommand(() -> {
		}, () -> Robot.cont.climber.io.override(-1), interrupted -> Robot.cont.climber.io.override(0), () -> false));
		this.climberOverrideRaise.whileTrue(new FunctionalCommand(() -> {
		}, () -> Robot.cont.climber.io.override(1), interrupted -> Robot.cont.climber.io.override(0), () -> false));

		this.initializeClimber
			.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.INITIALIZE)))
			.onFalse(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setClimberIntent(ClimberGoal.IDLE)));

		this.intakeOut.whileTrue(new FunctionalCommand(() -> {
		},
			() -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Reverse),
			interrupt -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Halt),
			() -> false
		));
		this.intakeIn.whileTrue(new FunctionalCommand(() -> {
		},
			() -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Forward),
			interrupt -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Halt),
			() -> false
		));

		this.fixedShoot
			.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setShooterIntent(ShooterGoal.SHOOT_FIXED)))
			.onFalse(new edu.wpi.first.wpilibj2.command.InstantCommand(
				() -> Robot.cont.superstructure.resolver.setShooterIntent(ShooterGoal.HOME)));
	}
}
