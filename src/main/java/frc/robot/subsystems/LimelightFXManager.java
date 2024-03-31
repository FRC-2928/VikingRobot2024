package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightFX.*;
import frc.robot.subsystems.LimelightFX.Behavior.*;
import frc.robot.subsystems.LimelightFX.Module.*;

public class LimelightFXManager {
	public final LimelightFX fx = new LimelightFX();

	public final LimelightFX.Module grid = this.fx.module(Geometry.grid, Rotation.R0);
	// public final LimelightFX.Module[] strips = new LimelightFX.Module[] {
	// 	// we have 8 strips, however since the strips are daisy chained and we have 2 directly attached, we splice them together as one strip
	// 	this.fx.module(Geometry.strip.size(32, 1), Rotation.R0),
	// 	this.fx.module(Geometry.strip.size(32, 1), Rotation.R0),

	// 	this.fx.module(Geometry.strip.size(32, 1), Rotation.R0),
	// 	this.fx.module(Geometry.strip.size(32, 1), Rotation.R0), };

	public final Behavior<?> behDisabledStrips = this.fx.behavior(BlinkBehavior.class).solidColor(Color.WHITE);
	public final Behavior<?> behDisabledGrid = this.fx.behavior(ImageBehavior.class).of("disabled");

	public final Behavior<?> behDisabledDiagnosticsIssue = this.fx
		.behavior(BlinkBehavior.class)
		.solidColor(Color.RED)
		.cfg(beh -> beh.timeOffA.set(0.25));

	public final Behavior<?> behAuto = this.fx.behavior(BlinkBehavior.class);
	public final Behavior<?> behTeleop = this.fx.behavior(ImageBehavior.class);
	public final Behavior<?> behHoldingNote = this.fx
		.behavior(BlinkBehavior.class)
		.solidColor(Constants.LimelightFX.Colors.note)
		.on(this.grid, 0);

	@SuppressWarnings({ "resource" })
	public LimelightFXManager() {
		if(!Constants.LimelightFX.enabled) return;

		{
			final BlinkBehavior beh = (BlinkBehavior) this.behHoldingNote;

			beh.colorA.set(Constants.LimelightFX.Colors.note);

			beh.timeOnA.set(1.0);
			beh.timeOffA.set(0.25);
			beh.timeBetween.set(0.0);
			beh.timeOnB.set(0.0);
			beh.timeOffB.set(0.0);
			beh.timeRepeat.set(0.0);

			beh.blinkCountA.set(1);
			beh.blinkCountB.set(1);
			beh.repeatCount.set(0);

			beh.fadeInA.set(0.0);
			beh.fadeOutA.set(0.0);
			beh.fadeInB.set(0.0);
			beh.fadeOutB.set(0.0);
		}

		this.fx.selector(() -> {
			if(Robot.cont.shooter.inputs.holdingNote) return this.behHoldingNote;
			else return null;
		});

		this.fx.initialize(() -> {
			final SerialPort serial = new SerialPort(115200, SerialPort.Port.kUSB1);
			serial.reset();
			serial.setTimeout(1);
			serial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
			return str -> {
				Logger.recordOutput("LLFX/Command", Timer.getFPGATimestamp() + ": " + str);
				return serial.writeString(str) != 0;
			};
		});
	}
}