package frc.robot.commands.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class RunTests extends Command {
	public static abstract class Test {
		public abstract String name();

		public abstract void begin();

		public abstract void execute();

		public abstract void end(boolean pass);

		public void pass(final String msg) {
			this.end(true);
			Robot.cont.diag.chirp(true);
			System.out.println("[DIAGNOSTICS] '" + this.name() + "' PASS: " + msg);
		}

		public void fail(final String msg) {
			this.end(false);
			Robot.cont.diag.chirp(false);
			System.out.println("[DIAGNOSTICS] '" + this.name() + "' PASS: " + msg);
		}
	}

	public RunTests() {

	}
}
