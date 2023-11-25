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
    //**TODO pLimit may be negative - why use abs and then multiply? Use straight Range.clip??
    // pLimit really is the *lower* limit ...
    public static double clip(double pValue, double pLimit) {
        return Range.clip(Math.abs(pValue), pLimit, 1.0) * (pValue < 0 ? -1 : 1);
    }

    //**TODO 11/21/2023 On a strafe you can't apply the same "steer"
    // factor to the left front and left rear motors - they are turning in opposite
    // directions!
    public static EnumMap<FTCRobot.MotorId, Double> updateDriveTrainVelocity(EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> pCurrentMotorData,
                                                                             double pSteer, double pRampDownFactor) {
        EnumMap<FTCRobot.MotorId, Double> newVelocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        double clippedVelocity;
        for (Map.Entry<FTCRobot.MotorId, AutoDrive.DriveMotorData> oneMotorEntry : pCurrentMotorData.entrySet()) {
            FTCRobot.MotorId motorId = oneMotorEntry.getKey();
            AutoDrive.DriveMotorData motorData = oneMotorEntry.getValue();

            switch (motorId) {
                case LEFT_FRONT_DRIVE:
                case LEFT_BACK_DRIVE: {
                    double newLeftVelocity = (motorData.initialVelocity * pRampDownFactor) - pSteer;
                    clippedVelocity = motorData.clipUpdatedVelocity(newLeftVelocity);
                    newVelocityMap.put(motorId, clippedVelocity);
                    break;
                }
                case RIGHT_FRONT_DRIVE:
                case RIGHT_BACK_DRIVE: {
                    double newRightVelocity = (motorData.initialVelocity * pRampDownFactor) + pSteer;
                    clippedVelocity = motorData.clipUpdatedVelocity(newRightVelocity);
                    newVelocityMap.put(motorId, clippedVelocity);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
            }
        }

        return newVelocityMap;
    }
}
