package org.firstinspires.ftc.teamcode.auto.vision;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.AngleDistance;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.EnumSet;

public class AprilTagUtils {

    private static final String TAG = AprilTagUtils.class.getSimpleName();

    private static final double STRAFE_LEFT = 90.0;
    private static final double STRAFE_RIGHT = -90.0;

    // For validation of the AprilTags on the BLUE side backdrop.
    private static final EnumSet<RobotConstantsCenterStage.AprilTagId> blueBackdropAprilTags =
            EnumSet.of(RobotConstantsCenterStage.AprilTagId.TAG_ID_1,
                    RobotConstantsCenterStage.AprilTagId.TAG_ID_2,
                    RobotConstantsCenterStage.AprilTagId.TAG_ID_3);

    // For validation of the AprilTags on the RED side backdrop.
    private static final EnumSet<RobotConstantsCenterStage.AprilTagId> redBackdropAprilTags =
            EnumSet.of(RobotConstantsCenterStage.AprilTagId.TAG_ID_4,
                    RobotConstantsCenterStage.AprilTagId.TAG_ID_5,
                    RobotConstantsCenterStage.AprilTagId.TAG_ID_6);

    // Call this method if the target AprilTag on the backstop could not
    // be located but at least one if its near neighbors in the same set
    // of three could be located. This method infers the location of the
    // target AprilTag from the position of its neighbor and returns the
    // distance and angle to the target AprilTag.

    public static AngleDistance inferAprilTag(RobotConstantsCenterStage.AprilTagId pTargetId,
                                              RobotConstantsCenterStage.AprilTagId pRecognizedId,
                                              double pDistanceToRecognizedId, double pAngleToRecognizedId) {
        if (pTargetId == pRecognizedId) // the caller shouldn't do this
            return new AngleDistance(pAngleToRecognizedId, pDistanceToRecognizedId); // so return the input

        // Validate that the target id and the recognized id belong to the
        // backdrop of the same alliance.
        if (!((blueBackdropAprilTags.contains(pTargetId) && blueBackdropAprilTags.contains(pRecognizedId)) ||
                (redBackdropAprilTags.contains(pTargetId) && redBackdropAprilTags.contains(pRecognizedId))))
            throw new AutonomousRobotException(TAG, "Target and recognized AprilTag ids do not belong to the same alliance");

        return calculator(pTargetId.getNumericId(), pRecognizedId.getNumericId(),
                pDistanceToRecognizedId, pAngleToRecognizedId);
    }

    // Returns the inferred angle and inferred distance to the target AprilTag.
    // Credit to Notre Dame students Brandon Lim for the calculations and Sotiris
    // Artenos for the IntelliJ test harness.
    private static AngleDistance calculator(int targetAprilTag, int givenAprilTag, double distanceGivenApril, double angleGivenApril) {

        int sign = targetAprilTag < givenAprilTag ? -1 : 1;
        int endsign = targetAprilTag > givenAprilTag ? -1 : 1;

        //Math shown in the notebook

        angleGivenApril = sign * Math.toRadians(angleGivenApril);
        double c = distanceGivenApril * Math.sin(angleGivenApril);
        double f = (Math.abs(targetAprilTag - givenAprilTag) * 6) - c;
        double d = distanceGivenApril * Math.cos(angleGivenApril);

        double newAngleApril = Math.atan(f / d);
        newAngleApril = endsign * Math.toDegrees(newAngleApril);
        double distanceFive = Math.sqrt(d * d + f * f);

        return new AngleDistance(newAngleApril, distanceFive);
    }

    //**TODO adapt and correct comments ...
    // Method that adjusts the distance of the strafe depending
    // on the AprilTag. The returned angle (90.0 degrees or -90.0 degrees)
    // is from the point of view of an observer facing the robot from the
    // center of the field. The returned distance is the positive distance
    // to strafe.
    // From the same point of view give the strafeAdjustment
    // method the number of inches that the center of the robot
    // is left (positive) or right (negative) of the AprilTag.
    // Mke the sign of the number of inches the same as the
    // inverse of the sign of the angle from the center of the
    // robot to the AprilTag.

    // The method strafeAdjustment is used for positions F4 and A4.
    // It is used for ending on the right or left side of the backdrop. It moves the robot to
    // the edge of the backdrop in order to drop the yellow pixel.
    public static AngleDistance strafeAdjustment(int aprilTag, double distanceToStrafe, double outsideAdjustment) {
        // for center april tags
        // no change is needed for center april tags
        if (aprilTag == 2 || aprilTag == 5) {
            if (distanceToStrafe >= 0) {
                return new AngleDistance(STRAFE_RIGHT, distanceToStrafe);
            } else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe));
            }
        }

        // for left april tags
        else if (aprilTag == 1 || aprilTag == 4) {

            // left of april tag
            if (distanceToStrafe >= 0) {
                // not far enough to the left from april tag
                // if robot is not far enough to the left the robot has to move a small amount more left
                if (distanceToStrafe - outsideAdjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe - outsideAdjustment));
                }

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be outsideAdjustment inches from april tag
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceToStrafe - outsideAdjustment));
                }
            }

            // right of april tag
            // has to move more left to reach outsideAdjustment inches from april tag
            else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(Math.abs(distanceToStrafe) + outsideAdjustment));
            }
        }

        // for right april tags
        else {

            // left of april tag
            // has to move more right to reach outsideAdjustment inches from april tag
            if (distanceToStrafe >= 0) {
                return new AngleDistance(STRAFE_RIGHT, Math.abs(Math.abs(distanceToStrafe) + outsideAdjustment));
            }

            // right of april tag
            else {

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be outsideAdjustment inches from april tag
                if (distanceToStrafe + outsideAdjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe + outsideAdjustment));
                }

                // not far enough to the left from april tag
                // if robot is not far enough to the right the robot has to move a small amount more right
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceToStrafe + outsideAdjustment));
                }
            }
        }
    }

    // The method yellowPixelAdjustment is used for positions F2 and A2.
    // It is used in order to move the robot a small distance left or right
    // in order to place the yellow pixel in the other slot on the backdrop.
    public static AngleDistance yellowPixelAdjustment(int aprilTag, double distanceToStrafe, RobotConstantsCenterStage.BackdropPixelOpenSlot openSlot, double yellowPixelAdjustment, double outsideAdjustment) {
        if (distanceToStrafe >= 0) {
            switch (openSlot) {
                case LEFT: {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceToStrafe - yellowPixelAdjustment));
                }
                case RIGHT: {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceToStrafe + yellowPixelAdjustment));
                }
                case ANY_OPEN_SLOT: {
                    return strafeAdjustment(aprilTag, distanceToStrafe, outsideAdjustment);
                }
                default: {
                    return new AngleDistance(STRAFE_RIGHT, distanceToStrafe);
                }
            }
        } else {
            switch (openSlot) {
                case LEFT: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe + yellowPixelAdjustment));
                }
                case RIGHT: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe - yellowPixelAdjustment));
                }
                case ANY_OPEN_SLOT: {
                    return strafeAdjustment(aprilTag, distanceToStrafe, outsideAdjustment);
                }
                default: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceToStrafe));
                }
            }
        }
    }

}
