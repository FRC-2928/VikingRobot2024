package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public Shooter() {
        this.flywheels.setNeutralMode(NeutralModeValue.Coast);
        this.flywheels.setInverted(true);
        this.launcher.setNeutralMode(NeutralMode.Brake);
        this.launcher.setInverted(true);
    }

    public final TalonFX flywheels = new TalonFX(24, "canivore");
    public final TalonSRX launcher = new TalonSRX(6);
}
