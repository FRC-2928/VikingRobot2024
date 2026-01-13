package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;

public class STalonFX extends TalonFX {
	public STalonFX(final int id, final String bus) { super(id, bus); }

	public STalonFX(final int id) { super(id); }

	public boolean stalling() {
		return false; // todo
	}
}
