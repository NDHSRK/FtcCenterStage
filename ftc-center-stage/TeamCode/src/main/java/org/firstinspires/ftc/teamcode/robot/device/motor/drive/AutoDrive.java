package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;

import java.util.EnumMap;

// For managing the motors during a straight run in Autonomous.
public class AutoDrive {
    private static final String TAG = AutoDrive.class.getSimpleName();

    private final EnumMap<FTCRobot.MotorId, DriveMotorData> allDriveMotors = new EnumMap<>(FTCRobot.MotorId.class);

    //**TODO Incorrect: for all straight-line movements we use
    // RUN_WITH_ENCODER, setTargtePosition, RUN_TO_POSITION, and
    // setVelocity.
    //**TODO Delete For a straight run in Autonomous we always use RUN_WITH_ENCODER
    // with velocity levels with or without RUN_TO_POSITION.

    // Directional velocity levels, including x, y, and rotational,
    // make sense in TeleOp where the driver can stop the robot
    // using visual cues. But in autonomous we use distance
    // (converted to encoder clicks for each motor) to know when a
    // movement is complete. To ask the user to predict click counts
    // in the presence of rotation is not practical. Full freedom of
    // movement in autonomous is best left to a package like
    // RoadRunner.
    @SuppressLint("DefaultLocale")
    public AutoDrive(double pFtcAngle, double pVelocity, double pDominantMotorVelocityLimit) {

        double dominantVelocityLimit = Math.abs(pDominantMotorVelocityLimit);

        // To ensure that the trigonometry produces values that match the
        // FTC convention (CCW 0 to +180 not inclusive, CW 0 to -180
        // inclusive), it is necessary to invert the FTC angle and
        // then convert it to a 0..360 CW range.
        double ftcAngle = DEGREES.normalize(pFtcAngle); // ensure correct range
        double angle360 = Headings.normalize(ftcAngle * -1, 0, 360);

        // Get the directional values.
        double directionX = Math.sin(Math.toRadians(angle360));
        double directionY = Math.cos(Math.toRadians(angle360));

        // Calculate the unclipped, unfactored directional velocity
        // for the mecanum drive motors. These values are used to
        // set the initial velocity and also determine the direction
        // signum below.
        double lfv = directionY + directionX;
        double rfv = directionY - directionX;
        double lbv = directionY - directionX;
        double rbv = directionY + directionX;

        // Determine which motors have the dominant velocity values.
        // The sign of the velocity for each motor can be used to
        // to determine the sign of the target click count for
        // RUN_TO_POSITION. Also, only the dominant motors need to
        // be checked for isBusy() during RUN_TO_POSITION.

        // Take a shortcut for all angles evenly divisible by 90 degrees.
        // For forward (0 degrees), backward (-180 degrees), strafe left
        // (90 degrees), and strafe right (-90 degrees) all motors will be
        // in play, all will be dominant, and all will have the same velocity.
        if (angle360 % 90.0 == 0.0) {
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData(lfv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData(rfv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData(lbv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData(rbv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            return;
        }

        // For all other angles, two of the four motors will be dominant
        // and will both have the same raw unclipped, unfactored directional
        // velocity as each other. The other two motors will be subordinate
        // and will both have the same raw velocity as each other. The
        // dominant velocity will always be greater than the subordinate
        // velocity.
        double subordinateVelocityLimit = 0.0;
        if (Math.abs(lfv) > Math.abs(rfv)) {
            // left front and right back are dominant
            double dominantRawVelocityLFRB = Math.abs(lfv);

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.LEFT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData(lfv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.RIGHT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData(rbv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));

            // Make the subordinate velocity proportional to the dominant velocity.
            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.RIGHT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData((dominantRawVelocityLFRB <= 1.0 ? rfv : rfv/dominantRawVelocityLFRB), pVelocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));

            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.LEFT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData((dominantRawVelocityLFRB <= 1.0 ? lbv : lbv/dominantRawVelocityLFRB), pVelocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));
        }
        else {
            // right front and left back are dominant
            double dominantRawVelocityRFLB = Math.abs(rfv);

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.RIGHT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData(rfv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
 
            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.LEFT_BACK_DRIVE); 
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData(lbv, pVelocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
 
            // Make the subordinate velocity proportional to the dominant velocity.
            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.LEFT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData((dominantRawVelocityRFLB <= 1.0 ? lfv : lfv/dominantRawVelocityRFLB), pVelocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));

            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.RIGHT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData((dominantRawVelocityRFLB <= 1.0 ? rbv : rbv/dominantRawVelocityRFLB), pVelocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));
        }
    }

    public EnumMap<FTCRobot.MotorId, DriveMotorData> getDriveMotorData() {
        return allDriveMotors;
    }

    //**TODO This comment is not correct and neither is the implementation!
    // Very important!
    // For motors set to RUN_TO_POSITION, the SDK does not look at
    // the sign of the velocity. However, for the PID control to
    // work correctly *[original] we do need the sign of the
    // velocity.[end original]* -> **NOT EXACTLY TRUE** - we need
    // to look at the sign of the velocity *after* the PID and its
    // "steer" value have been applied and never allow the final
    // velocity to fall below zero (for subordinate motors) or
    // below the minimum motor velocity (for dominant motors).

    public static class DriveMotorData {
        private static final String TAG = DriveMotorData.class.getSimpleName();

        public final int directionSignum;
        public final DriveTrainConstants.MotorRank motorRank;
        private final double velocityLimit;
        public final double initialVelocity;

        @SuppressLint("DefaultLocale")
        public DriveMotorData(double pDirectionalVelocity, double pVelocity, double pVelocityLimit,
                              DriveTrainConstants.MotorRank pMotorRank) {
            directionSignum = (int) Math.signum(pDirectionalVelocity);
            velocityLimit = pVelocityLimit;
            motorRank = pMotorRank;

            RobotLogCommon.vv(TAG, "Directional velocity " + String.format("%.3f", pDirectionalVelocity));
            RobotLogCommon.vv(TAG, "Requested velocity " + String.format("%.3f", pVelocity));
            
            double clippedDirectionalVelocity = MotionUtils.clip(pDirectionalVelocity, velocityLimit);
            RobotLogCommon.vv(TAG, "Clipped directional velocity " + String.format("%.3f", clippedDirectionalVelocity));
            
            initialVelocity = MotionUtils.clip(clippedDirectionalVelocity * pVelocity, velocityLimit);
            RobotLogCommon.vv(TAG, "Clipped initial velocity " + initialVelocity);
        }

        public double clipUpdatedVelocity(double pVelocity) {
            return MotionUtils.clip(pVelocity, velocityLimit);
        }
    }

}



