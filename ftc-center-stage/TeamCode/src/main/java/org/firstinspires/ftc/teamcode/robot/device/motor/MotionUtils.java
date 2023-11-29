package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.AutoDrive;

import java.util.EnumMap;
import java.util.Map;

public class MotionUtils {

    private static final String TAG = MotionUtils.class.getSimpleName();

    // Clip velocity or power.
    // Do not let the absolute value of the velocity or power go below the lower limit
    // or above the fixed maximum of 1.0. Return a value with the same sign as the
    // input argument.
    //**TODO Make two different clipping methods: one for RTP velocity (which may never
    // fall below zero but may be zero for subordinate motors) and one for RWE, i.e.
    // turning - with velocity that ranges between -1.0 and +1.0.
    public static double clip(double pValue, double pLimit) {
        return Range.clip(Math.abs(pValue), pLimit, 1.0) * (pValue < 0 ? -1 : 1);
    }

    public static EnumMap<FTCRobot.MotorId, Double> updateDriveTrainVelocity(EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> pCurrentMotorData,
                                                                             double pAngle, double pSteer, double pRampDownFactor) {
        EnumMap<FTCRobot.MotorId, Double> newVelocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        double clippedVelocity;
        for (Map.Entry<FTCRobot.MotorId, AutoDrive.DriveMotorData> oneMotorEntry : pCurrentMotorData.entrySet()) {
            FTCRobot.MotorId motorId = oneMotorEntry.getKey();
            AutoDrive.DriveMotorData motorData = oneMotorEntry.getValue();

            // If the angle is 0.0 or -180.0 then the robot is moving forward
            // or backward so apply the same correction to the two left side
            // motors and the inverse of the correction to the right side motors.
            if (pAngle == 0.0 || pAngle == -180.0) {
                switch (motorId) {
                    case LEFT_FRONT_DRIVE:
                    case LEFT_BACK_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
                        break;
                    }
                    case RIGHT_FRONT_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
                }

                continue;
            }

            // If the angle is 90.0 or -90.0 then the robot is strafing to
            // the left or right and the application of the steering correction
            // varies.
            if (pAngle == 90.0) { // strafe left?
                switch (motorId) {
                    case LEFT_FRONT_DRIVE:
                    case RIGHT_FRONT_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    case LEFT_BACK_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
                }

                continue;
            }

            if (pAngle == -90.0) { // strafe right?
                switch (motorId) {
                    case LEFT_FRONT_DRIVE:
                    case RIGHT_FRONT_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
                        break;
                    }
                    case LEFT_BACK_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        setRampedDownVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
                }

                continue;
            }

            //**TODO For all other angles the PID corrections are tricky.
            // So at this point just return the ramped-down velocity.
            setRampedDownVelocity(motorId, motorData, pRampDownFactor, 0, newVelocityMap);
        }

        return newVelocityMap;
    }

    private static void setRampedDownVelocity(FTCRobot.MotorId pMotorId, AutoDrive.DriveMotorData pMotorData,
                                    double pRampDownFactor, double pSteer,
                                    EnumMap<FTCRobot.MotorId, Double> pVelocityMap) {
        double newVelocity = (pMotorData.initialVelocity * pRampDownFactor) + pSteer;
        double clippedVelocity = pMotorData.clipUpdatedVelocity(newVelocity);
        pVelocityMap.put(pMotorId, clippedVelocity);
    }

}
