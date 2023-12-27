package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;

import java.util.EnumMap;
import java.util.Objects;

// Linear power ramp-down that starts at a user-specified number of inches
// from the target (converted to motor clicks).

// Velocity values for all motors will be the same only when the angle of motion
// is evenly divisible by 90. In this case all 4 motors are dominant. Otherwise,
// e.g. when the robot is moving in a straight line at a 45-degree angle, there
// will always be two dominant motors and two subordinate motors.

public class StraightDriveRampdown {

    public static final String TAG = StraightDriveRampdown.class.getSimpleName();
    private final FTCRobot robot;
    private final int rampDownClicks;
    private final EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> allDriveMotors;

    private final FTCRobot.MotorId dominantMotorId;
    private double previousDominantVelocity;

    @SuppressLint("DefaultLocale")
    public StraightDriveRampdown(FTCRobot pRobot, int pRampDownClicks,
                                 EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> pAllDriveMotors, FTCRobot.MotorId pDominantMotorId) {
        robot = pRobot;
        rampDownClicks = Math.abs(pRampDownClicks);
        allDriveMotors = pAllDriveMotors;
        dominantMotorId = pDominantMotorId;
        previousDominantVelocity = Math.abs(Objects.requireNonNull(allDriveMotors.get(dominantMotorId)).initialVelocity);

        RobotLogCommon.d(TAG, "Ramp down initial velocity of " + String.format("%.2f", previousDominantVelocity) +
                " starting at " + rampDownClicks + " clicks from the target");
    }

    // Returns the ramp down factor, which starts at 1.0 when the robot is running
    // at the velocity specified in RobotAction.xml, and decreases as the robot nears
    // its target.
    @SuppressLint("DefaultLocale")
    public double rampDown(int pRemainingClicks, double pAngle, double pSteer, boolean pLogVV) {

        EnumMap<FTCRobot.MotorId, Double> newVelocityMap;

        // Sanity check
        if (pRemainingClicks <= 0.0)
            throw new AutonomousRobotException(TAG, "Remaining clicks " + pRemainingClicks +
                    " may not be <= 0");

        int remainingCLicks = Math.abs(pRemainingClicks); // abs for consistency

        // Sanity check: the remaining number of clicks must be <= the point at which the
        // ramp down is to start.
        if (remainingCLicks > rampDownClicks)
            throw new AutonomousRobotException(TAG, "Remaining clicks " + remainingCLicks +
                    " is > ramp down start point of " + rampDownClicks);

        // Simple linear ramp-down. Cast to double is necessary, otherwise result
        // can be 0 when you don't want it.
        double rampDownFactor = Math.abs(pRemainingClicks / (double) rampDownClicks);

        // Running continuously results in frequent small updates to motor velocity.
        // It's better to step the velocity down by this logic: if the absolute value
        // of the current velocity of any dominant motor is equal to the minimum
        // velocity OR the difference between the current velocity and the previous
        // velocity is less than the minimum velocity step, e.g. .05, then do NOT
        // update the motor velocity.
        double currentDominantVelocity =
                Math.abs(MotionUtils.clip(Objects.requireNonNull(allDriveMotors.get(dominantMotorId)).initialVelocity * rampDownFactor, DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY));

        if (pLogVV)
            RobotLog.vv(TAG, "Next candidate for velocity ramp-down " + String.format("%.2f", currentDominantVelocity) +
                    " using factor " + String.format("%.3f", rampDownFactor) +
                    ", previous " + String.format("%.2f", previousDominantVelocity) +
                    ", difference " + String.format("%.3f", Math.abs(currentDominantVelocity - previousDominantVelocity)));

        if (currentDominantVelocity == DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY || Math.abs(currentDominantVelocity - previousDominantVelocity) < DriveTrainConstants.MINIMUM_DRIVE_POWER_STEP)
            return rampDownFactor;

        previousDominantVelocity = currentDominantVelocity;
        newVelocityMap = MotionUtils.updateDriveTrainVelocity(allDriveMotors, pAngle, pSteer, rampDownFactor);
        robot.driveTrain.runAtVelocityAll(newVelocityMap);

        if (pLogVV)
            RobotLog.vv(TAG, "Straight line velocity ramped down to lf " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.LEFT_FRONT_DRIVE)) +
                    ", rf " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.RIGHT_FRONT_DRIVE)) +
                    ", lb " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.LEFT_BACK_DRIVE)) +
                    ", rb " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.RIGHT_BACK_DRIVE)));

        return rampDownFactor;
    }

}
