package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;

import java.util.EnumMap;

// Linear power ramp-down that starts at a user-specified number of degrees.
public class TurnRampdown {

    public static final String TAG = "RampDownTurn";
    private final FTCRobot robot;
    private final double rampDownDegrees;
    private double previousPower;
    private final EnumMap<FTCRobot.MotorId, Double> currentPowerMap;

    @SuppressLint("DefaultLocale")
    public TurnRampdown(FTCRobot pRobot, double pRampDownDegrees, EnumMap<FTCRobot.MotorId, Double> pCurrentPowerMap) {
        robot = pRobot;
        rampDownDegrees = Math.abs(pRampDownDegrees);
        currentPowerMap = pCurrentPowerMap;

        // Power values for all motors are the same (although their signs may be different).
        // Choose any motor to set the previous power.
        previousPower = Math.abs(currentPowerMap.get(FTCRobot.MotorId.LEFT_FRONT_DRIVE));

        RobotLogCommon.d(TAG, "Ramp down turn power starting at " + String.format("%.2f", rampDownDegrees) +
                " degrees remaining and " + String.format("%.2f", previousPower) + " power");
    }

    @SuppressLint("DefaultLocale")
    public void rampDown(double pRemainingAngle) {
        double remainingAngle = Math.abs(pRemainingAngle); // take no chances

        EnumMap<FTCRobot.MotorId, Double> newPowerMap = new EnumMap<>(FTCRobot.MotorId.class);

        // Sanity check: the remaining angle must be <= the point at which the ramp down
        // is to start.
        if (remainingAngle > rampDownDegrees)
            throw new AutonomousRobotException(TAG, "Remaining angle of " + String.format("%.2f", remainingAngle) +
                    " is > ramp down start point of " + String.format("%.2f", rampDownDegrees));

        // Simple linear ramp-down.
        double rampDownFactor = remainingAngle / rampDownDegrees;

        // Running continuously results in frequent small updates to motor power. It's
        // better to step the power down by this logic: if the absolute value of the
        // current power (of any motor, all are the same) is equal to the minimum
        // power OR the difference between the current power and the previous power
        // is less than the minimum power step, e.g. .05, then do NOT update the power
        // to the motors.
        double currentPower = Math.abs(MotionUtils.clip(currentPowerMap.get(FTCRobot.MotorId.LEFT_FRONT_DRIVE) * rampDownFactor, DriveTrainConstants.MINIMUM_TURN_POWER));
        if (currentPower == DriveTrainConstants.MINIMUM_TURN_POWER || Math.abs(currentPower - previousPower) < DriveTrainConstants.MINIMUM_TURN_POWER_STEP)
            return;

        previousPower = currentPower;
        double clippedPower;
        for (EnumMap.Entry<FTCRobot.MotorId, Double> oneMotor : currentPowerMap.entrySet()) {
            clippedPower = MotionUtils.clip(oneMotor.getValue() * rampDownFactor, DriveTrainConstants.MINIMUM_TURN_POWER);
            newPowerMap.put(oneMotor.getKey(), clippedPower);
        }

        robot.driveTrain.setPowerAll(newPowerMap);

        RobotLogCommon.vv(TAG, "Turn power lf " + String.format("%.2f", newPowerMap.get(FTCRobot.MotorId.LEFT_FRONT_DRIVE)) +
                ", rf " + String.format("%.2f", newPowerMap.get(FTCRobot.MotorId.RIGHT_FRONT_DRIVE)) +
                ", lb " + String.format("%.2f", newPowerMap.get(FTCRobot.MotorId.LEFT_BACK_DRIVE)) +
                ", rb " + String.format("%.2f", newPowerMap.get(FTCRobot.MotorId.RIGHT_BACK_DRIVE)));
    }

}
