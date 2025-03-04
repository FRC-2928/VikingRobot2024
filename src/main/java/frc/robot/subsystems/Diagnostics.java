package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.net.InetAddress;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.Alert;

public class Diagnostics extends SubsystemBase {
	public final class Release extends Command {
		public Release() { this.addRequirements(RobotContainer.getInstance().drivetrain); }

		@Override
		public void initialize() {
			for(final SwerveModule module : RobotContainer.getInstance().drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Coast);
				io.drive.setNeutralMode(NeutralModeValue.Coast);
			}

			Diagnostics.this.chirp(1000, 100);
			Diagnostics.this.chirp(800, 500);
		}

		@Override
		public void end(final boolean interrupted) {
			for(final SwerveModule module : RobotContainer.getInstance().drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Brake);
				io.drive.setNeutralMode(NeutralModeValue.Brake);
			}

			Diagnostics.this.chirp(800, 100);
			Diagnostics.this.chirp(1000, 500);
		}

		@Override
		public boolean runsWhenDisabled() { return true; }
	}

	private static final class Chirp {
		public Chirp(final int freq, final int ms) {
			this.freq = freq;
			this.ms = ms;
		}

		public int freq;
		public int ms;
		public long start = 0;

		// public void play() { RobotContainer.getInstance().fxm.fx.sound(LimelightFX.WaveForm.Square, this.freq, this.ms, 0, 1); }
	}

	private static final byte suffixRadio = 1;
	private static final byte suffixDS = 1;
	private static final byte suffixLLNote = 1;
	private static final byte suffixLLShooter = 1;
	private static final byte suffixLLRear = 1;

	public Diagnostics() {
		try {
			this.loggedDashboardChooserSelectedValue = LoggedDashboardChooser.class.getDeclaredField("selectedValue");
			this.loggedDashboardChooserSelectedValue.setAccessible(true);
		} catch(final Exception e) {
			throw new Error(e);
		}
	}

	private final ArrayList<Chirp> chirps = new ArrayList<>();
	private final Field loggedDashboardChooserSelectedValue;

	public final DigitalInput releaseInput = new DigitalInput(0);
	public final Trigger release = new Trigger(() -> DriverStation.isDisabled() && !this.releaseInput.get());

	public volatile LimelightFX.Behavior<?> active;

	public void chirp(final boolean good) { this.chirps.add(new Chirp(good ? 500 : 125, 500)); }

	public void chirp(final int freq, final int ms) { this.chirps.add(new Chirp(freq, ms)); }

	public void configureControls() {
		this.release.whileTrue(this.new Release());

		SmartDashboard
			.putData("Diagnostics/C-Stop", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

	}

	@Override
	public void periodic() {
		if(this.chirps.size() > 0) {
			final Chirp chirp = this.chirps.get(0);
			if(chirp.start == 0) {
				chirp.start = Logger.getTimestamp();
				// chirp.play();
			}

			if(Logger.getRealTimestamp() - chirp.start >= chirp.ms) this.chirps.remove(0);
		}
	}

	private void run() {
		final Alert alertCanDeviceMissing = new Alert("Warnings", "CAN Devices Missing: <none>", Alert.AlertType.ERROR);
		final Alert alertMissingRadio = new Alert("Warnings", "Radio missing", Alert.AlertType.ERROR);
		final Alert alertMissingDs = new Alert("Warnings", "DS missing", Alert.AlertType.ERROR);
		final Alert alertMissingLlNote = new Alert("Warnings", "Note Limelight Missing", Alert.AlertType.ERROR);
		final Alert alertMissingLlShooter = new Alert("Warnings", "Shooter Limelight Missing", Alert.AlertType.ERROR);
		final Alert alertMissingLlRear = new Alert("Warnings", "Rear Limelight Missing", Alert.AlertType.ERROR);
		final Alert alertSetupJumperPresent = new Alert(
			"Warnings",
			"Setup Jumper is plugged in",
			Alert.AlertType.ERROR
		);

		final Alert alertInvalidAutoRoutine = new Alert(
			"Pre-Match",
			"Autonomous Routine is not ready for competition",
			Alert.AlertType.ERROR
		);
		final Alert alertShooterAngle = new Alert("Pre-Match", "Shooter is not upright", Alert.AlertType.ERROR);
		final Alert alertClimberHome = new Alert("Pre-Match", "Climber is not at home position", Alert.AlertType.ERROR);
		final Alert alertNotePossession = new Alert("Pre-Match", "Not possessing a note", Alert.AlertType.ERROR);
		final Alert alertBadVoltage = new Alert(
			"Pre-Match",
			"Battery Voltage is 0V, recommended to be 12V",
			Alert.AlertType.WARNING
		);

		while(true) {
			try {
				Thread.sleep(500);
			} catch(final Exception e) {
			}

			Logger.recordOutput("Diagnostics/Release", this.release.getAsBoolean());

			Logger.recordOutput("Diagnostics/Competition", DriverStation.isFMSAttached());

			final boolean setupJumperPresent = this.release.getAsBoolean();
			alertSetupJumperPresent.active = setupJumperPresent;

			final boolean invalidAutoRoutine;

			String name;
			try {
				name = (String) this.loggedDashboardChooserSelectedValue.get(RobotContainer.getInstance().autoChooser);
				if(name == null) name = "<none>";
				invalidAutoRoutine = !name.contains("[comp]");
			} catch(final Exception e) {
				throw new Error(e);
			}

			final boolean badVoltage = RobotController.getBatteryVoltage() < 12;

			alertInvalidAutoRoutine.text = "Autonomous routine '" + name + "' not ready for competition!";
			alertInvalidAutoRoutine.active = invalidAutoRoutine;
			alertBadVoltage.text = "Battery Voltage is "
				+ RobotController.getBatteryVoltage()
				+ "V, recommended to be at least 12V";
			alertBadVoltage.active = badVoltage;

			//final boolean ready = !(canDeviceMissing || missingRadio || missingLLNote || missingLLShooter || missingLLRear || missingDS || setupJumperPresent || invalidAutoRoutine || shooterAngle || badVoltage);

			//Logger.recordOutput("Diagnostics/ReadyForCompetition", ready);
		}
	}

	private final boolean reachable(final byte suffix, final int port) {
		try {
			return InetAddress.getByAddress(new byte[] { 10, 29, 28, suffix }).isReachable(8);
		} catch(final Exception e) {
			return false;
		}
	}

	private final <T> T firstNonNull(final T... params) {
		for(final T value : params)
			if(value != null) return value;

		return null;
	}
}
