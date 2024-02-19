package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;

import java.util.EnumMap;

// For managing the motors during a straight run in Autonomous.
public class AutoDrive {
    private static final String TAG = AutoDrive.class.getSimpleName();

    private final EnumMap<FTCRobot.MotorId, DriveMotorData> allDriveMotors = new EnumMap<>(FTCRobot.MotorId.class);

    // For all straight-line movements we use RUN_WITH_ENCODER, setTargetPosition,
    // RUN_TO_POSITION, and setVelocity.

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

        // Sanity: for RUN_TO_POSITION velocity is always positive - so
        // make sure the parameters pVelocity and pDominantMotorVelocityLimit
        // are positive.
        double velocity = Math.abs(pVelocity);
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

        // Calculate the unclipped, unfactored directional
        // velocity for each of the mecanum drive motors.
        // For example:
        // At 45, 135, 225, and 315 degrees two of the motors
        // will have a directional value of 0. For example,
        // sin 45 deg = 0.70710678118; cos = 0.70710678118
        // sin 135 deg = 0.70710678118; cos = -0.70710678118

        // These values are used to set the direction signum,
        // which eventually determines the sign of the click
        // counts for each motor in RUN_TO_POSITION and which
        // also determines the (always positive) initial velocity
        // for each motor.
        double lfv = directionY + directionX;
        double rfv = directionY - directionX;
        double lbv = directionY - directionX;
        double rbv = directionY + directionX;

        // Determine which motors have the dominant velocity values.
        // Only the dominant motors need to be checked for isBusy()
        // during RUN_TO_POSITION.

        // Take a shortcut for all angles evenly divisible by 90 degrees.
        // For forward (0 degrees), backward (-180 degrees), strafe left
        // (90 degrees), and strafe right (-90 degrees) all motors will be
        // in play, all will be dominant, and all will have the same velocity.
        if (angle360 % 90.0 == 0.0) {
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData(lfv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData(rfv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData(lbv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData(rbv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
            return;
        }

        // For all other angles, two of the four motors will be dominant
        // and will both have the same raw unclipped, unfactored velocity
        // as each other. The other two motors will be subordinate and
        // and will both have the same raw velocity as each other. The
        // dominant velocity will always be greater than the subordinate
        // velocity.
        double subordinateVelocityLimit = 0.0;
        if (Math.abs(lfv) > Math.abs(rfv)) {
            // left front and right back are dominant
            double dominantRawVelocityLFRB = Math.abs(lfv);

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.LEFT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData(lfv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.RIGHT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData(rbv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));

            // Make the subordinate velocity proportional to the dominant velocity.
            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.RIGHT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData((dominantRawVelocityLFRB <= 1.0 ? rfv : rfv/dominantRawVelocityLFRB), velocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));

            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.LEFT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData((dominantRawVelocityLFRB <= 1.0 ? lbv : lbv/dominantRawVelocityLFRB), velocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));
        }
        else {
            // right front and left back are dominant
            double dominantRawVelocityRFLB = Math.abs(rfv);

            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.RIGHT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, new DriveMotorData(rfv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
 
            RobotLogCommon.d(TAG, "Dominant motor " + FTCRobot.MotorId.LEFT_BACK_DRIVE); 
            allDriveMotors.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, new DriveMotorData(lbv, velocity, dominantVelocityLimit, DriveTrainConstants.MotorRank.DOMINANT));
 
            // Make the subordinate velocity proportional to the dominant velocity.
            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.LEFT_FRONT_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, new DriveMotorData((dominantRawVelocityRFLB <= 1.0 ? lfv : lfv/dominantRawVelocityRFLB), velocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));

            RobotLogCommon.d(TAG, "Subordinate motor " + FTCRobot.MotorId.RIGHT_BACK_DRIVE);
            allDriveMotors.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, new DriveMotorData((dominantRawVelocityRFLB <= 1.0 ? rbv : rbv/dominantRawVelocityRFLB), velocity, subordinateVelocityLimit, DriveTrainConstants.MotorRank.SUBORDINATE));
        }
    }

    public EnumMap<FTCRobot.MotorId, DriveMotorData> getDriveMotorData() {
        return allDriveMotors;
    }

    public static class DriveMotorData {
        private static final String TAG = DriveMotorData.class.getSimpleName();

        // The direction signum eventually determines the sign of the
        // click count for each motor used in RUN_TO_POSITION.
        public final int directionSignum;
        public final DriveTrainConstants.MotorRank motorRank;
        private final double velocityLimit;
        public final double initialVelocity;

        @SuppressLint("DefaultLocale")
        public DriveMotorData(double pDirectionalVelocity, double pVelocity, double pVelocityLimit,
                              DriveTrainConstants.MotorRank pMotorRank) {
            directionSignum = (int) Math.signum(pDirectionalVelocity);
            velocityLimit = Math.abs(pVelocityLimit); // sanity: must always be 0 or positive
            motorRank = pMotorRank;

            RobotLogCommon.vv(TAG, "Directional velocity " + String.format("%.3f", pDirectionalVelocity));
            RobotLogCommon.vv(TAG, "Requested velocity " + String.format("%.3f", pVelocity));
            
            double clippedDirectionalVelocity = MotionUtils.clipVelocity(pDirectionalVelocity, velocityLimit);
            RobotLogCommon.vv(TAG, "Clipped directional velocity " + String.format("%.3f", clippedDirectionalVelocity));

            // initialVelocity is guaranteed to be positive.
            initialVelocity = MotionUtils.clipVelocity(clippedDirectionalVelocity * pVelocity, velocityLimit);
            RobotLogCommon.vv(TAG, "Clipped initial velocity " + initialVelocity);
        }

        public double clipUpdatedVelocity(double pVelocity) {
            return MotionUtils.clipVelocity(pVelocity, velocityLimit);
        }
    }

}



