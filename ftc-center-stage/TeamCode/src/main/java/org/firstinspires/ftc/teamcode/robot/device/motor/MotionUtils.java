package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.AutoDrive;

import java.util.EnumMap;
import java.util.Map;

public class MotionUtils {

    private static final String TAG = MotionUtils.class.getSimpleName();

    // The lower limit for velocity is always 0 or positive so
    // this method guarantees that the return value will be 0 or
    // positive. The parameter pValue may be negative when a
    // directional velocity is passed in.
    public static double clipVelocity(double pValue, double pLowerLimit) {
        double value = Math.abs(pValue);
        double lowerLimit = Math.abs(pLowerLimit);
        return Range.clip(value, lowerLimit, 1.0);
    }

    // For use with power where the sign must be preserved since it
    // determines the direction of the motors.
    public static double clipPower(double pValue, double pLimit) {
        return Range.clip(Math.abs(pValue), pLimit, 1.0) * (pValue < 0 ? -1 : 1);
    }

    public static EnumMap<FTCRobot.MotorId, Double> updateDriveTrainVelocity(EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> pCurrentMotorData,
                                                                             double pAngle, double pSteer, double pRampDownFactor) {
        EnumMap<FTCRobot.MotorId, Double> newVelocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        for (Map.Entry<FTCRobot.MotorId, AutoDrive.DriveMotorData> oneMotorEntry : pCurrentMotorData.entrySet()) {
            FTCRobot.MotorId motorId = oneMotorEntry.getKey();
            AutoDrive.DriveMotorData motorData = oneMotorEntry.getValue();

            // If the angle is 0.0 or -180.0 then the robot is moving forward
            // or backward so apply the same correction to the two left side
            // motors and the inverse of the correction to the right side motors.
            // For example, if "steer" is negative then then robot is skewed
            // counter-clockwise. So to correct the skew for a straight-line run
            // you would increase the velocity of the left-side motors by the
            // inverse of the steer and decrease the velocity of the right-side
            // motors by the steer.
            if (pAngle == 0.0 || pAngle == -180.0) {
                switch (motorId) {
                    case LEFT_FRONT_DRIVE:
                    case LEFT_BACK_DRIVE: {
                        updateVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
                        break;
                    }
                    case RIGHT_FRONT_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        updateVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
                }

                continue;
            }

            //**TODO Re-test with vv logging. The increases/decreases in velocity
            // according to the value of "steer" look correct.
            // If the angle is 90.0 or -90.0 then the robot is strafing to
            // the left or right and the application of the steering correction
            // varies.
            if (pAngle == 90.0) { // strafe left?
                switch (motorId) {
                    case LEFT_FRONT_DRIVE:
                    case RIGHT_FRONT_DRIVE: {
                        updateVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    case LEFT_BACK_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        updateVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
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
                        updateVelocity(motorId, motorData, pRampDownFactor, -pSteer, newVelocityMap);
                        break;
                    }
                    case LEFT_BACK_DRIVE:
                    case RIGHT_BACK_DRIVE: {
                        updateVelocity(motorId, motorData, pRampDownFactor, pSteer, newVelocityMap);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid motor position " + motorId);
                }

                continue;
            }

            //**TODO For all other angles the PID corrections are tricky.
            // So at this point just return the ramped-down velocity.
            updateVelocity(motorId, motorData, pRampDownFactor, 0, newVelocityMap);
        }

        return newVelocityMap;
    }

    private static void updateVelocity(FTCRobot.MotorId pMotorId, AutoDrive.DriveMotorData pMotorData,
                                       double pRampDownFactor, double pSteer,
                                       EnumMap<FTCRobot.MotorId, Double> pVelocityMap) {
        double newVelocity = (pMotorData.initialVelocity * pRampDownFactor) + pSteer;
        double clippedVelocity = pMotorData.clipUpdatedVelocity(newVelocity);
        pVelocityMap.put(pMotorId, clippedVelocity);
    }

}
