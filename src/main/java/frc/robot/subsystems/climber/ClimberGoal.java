package frc.robot.subsystems.climber;

public enum ClimberGoal {
    /** Hold current position. Safe default. */
    IDLE,

    /** Raise by initializeRaiseDistance, then descend until home switch closes and encoder zeros. */
    INITIALIZE,

    /** Extend actuator to maximum height. Blocked if shooter is not at safe angle. */
    DEPLOY,

    /** Pull actuator down to position 0. */
    RETRACT,

    /** Motor output zero. */
    LOCKED
}
