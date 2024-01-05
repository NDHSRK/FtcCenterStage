package org.firstinspires.ftc.teamcode.auto.vision;

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

    // Returns a Pair where first is the inferred distance to the target AprilTag
    // and second is the inferred angle to the target AprilTag.
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

    public static AngleDistance strafeAdjustment(int aprilTag, double distanceFromCenter, double adjustment) {
        // for center april tags
        // no change is needed for center april tags
        if (aprilTag == 2 || aprilTag == 5) {
            if (distanceFromCenter >= 0) {
                return new AngleDistance(STRAFE_RIGHT, distanceFromCenter);
            } else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(distanceFromCenter));
            }
        }

        // for left april tags
        else if (aprilTag == 1 || aprilTag == 4) {

            // left of april tag
            if (distanceFromCenter >= 0) {
                // not far enough to the left from april tag
                // if robot is not far enough to the left the robot has to move a small amount more left
                if (distanceFromCenter - adjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceFromCenter - adjustment));
                }

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be adjustment inches from april tag
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceFromCenter - adjustment));
                }
            }

            // right of april tag
            // has to move more left to reach adjustment inches from april tag
            else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(Math.abs(distanceFromCenter) + adjustment));
            }
        }

        // for right april tags
        else {

            // left of april tag
            // has to move more right to reach adjustment inches from april tag
            if (distanceFromCenter >= 0) {
                return new AngleDistance(STRAFE_RIGHT, Math.abs(Math.abs(distanceFromCenter) + adjustment));
            }

            // right of april tag
            else {

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be adjustment inches from april tag
                if (distanceFromCenter + adjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(distanceFromCenter + adjustment));
                }

                // not far enough to the left from april tag
                // if robot is not far enough to the right the robot has to move a small amount more right
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(distanceFromCenter + adjustment));
                }
            }
        }
    }

}
