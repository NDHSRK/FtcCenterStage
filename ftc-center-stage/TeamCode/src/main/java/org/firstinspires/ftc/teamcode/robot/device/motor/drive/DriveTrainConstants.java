package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

public class DriveTrainConstants {

    // Comes into play during motion in a straight line, especially when
    // the robot is moving at an angle.
    public enum MotorRank {
        DOMINANT, SUBORDINATE
    }

    public enum Direction {
        FORWARD, BACK
    }

    // Drive constants
    public static final double MINIMUM_DOMINANT_MOTOR_VELOCITY = 0.2; // minimum velocity to turn the wheels

    // PID constants
    public static final double P_DRIVE_COEFF = 0.06; // was 0.03; // Larger is more responsive, but also less stable
    public static final double I_DRIVE_COEFF = P_DRIVE_COEFF / 10;
    public static final double P_TURN_COEFF = 0.01;  // Larger is more responsive, but also less stable

    // When adjusting the velocity by the PID value do so in the following increment.
    public static final double MINIMUM_DRIVE_POWER_STEP = 0.025;

    // Turn constants
    public static final double MINIMUM_TURN_POWER = 0.2; // floor for power to the motors in a turn
    public static final double SKEW_THRESHOLD_DEGREES = 3.0;

    // When ramping down the power in a turn do so in the following increment.
    public static final double MINIMUM_TURN_POWER_STEP = 0.05;

    // If a turn is this close to its target, consider it done.
    public static final double TURN_THRESHOLD_DEGREES = 2.0; // was 1.0; // As tight as we can make it with the gyro

    public enum PostTurnHeading {
        RELATIVE_TO_START, RELATIVE_TO_CURRENT
    }

    public enum TurnNormalization {
        NORMALIZED, UNNORMALIZED
    }
}
