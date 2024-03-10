package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class Initialize extends Command {
	public Initialize() { this.addRequirements(Robot.cont.climber); }

	private boolean descending;
	private double start;

	@Override
	public void initialize() {
		this.descending = false;
		this.start = Robot.cont.climber.inputs.position;
	}

	@Override
	public void execute() {
		if(this.descending) {
			Robot.cont.climber.io.set(this.start - 1000);

			Logger.recordOutput("Climber/Initialize/State", "Descending");
		} else {
			Robot.cont.climber.io.set(this.start + 2);
			Logger.recordOutput("Climber/Initialize/Threshold", this.start + Constants.Climber.initializeRaiseDistance);
			if(Robot.cont.climber.inputs.position > this.start + Constants.Climber.initializeRaiseDistance)
				this.descending = true;

			Logger.recordOutput("Climber/Initialize/State", "Raising");
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.climber.io.offset(0);

		Logger.recordOutput("Climber/Initialize/State", "End");
	}

	@Override
	public boolean isFinished() { return Robot.cont.climber.inputs.home; }
}
