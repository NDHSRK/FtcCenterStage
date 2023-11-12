package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.EnumSet;

public class AprilTagUtils {

    private static String TAG = AprilTagUtils.class.getSimpleName();

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

    public static Pair<Double, Double> inferAprilTag(RobotConstantsCenterStage.AprilTagId pTargetId,
                                                     RobotConstantsCenterStage.AprilTagId pRecognizedId,
                                                     double pDistanceToRecognizedId, double pAngleToRecognizedId) {
        if (pTargetId == pRecognizedId) // the caller shouldn't do this
            return Pair.create(pDistanceToRecognizedId, pAngleToRecognizedId); // so return the input

        // Validate that the target id and the recognized id belong to the
        // backdrop of the same alliance.
        if (!((blueBackdropAprilTags.contains(pTargetId) && blueBackdropAprilTags.contains(pRecognizedId)) ||
                (redBackdropAprilTags.contains(pTargetId) && redBackdropAprilTags.contains(pRecognizedId))))
            throw new AutonomousRobotException(TAG, "Target and recognized AprilTag ids do not belong to the same alliance");

        return calculator(pTargetId.getNumericId(), pRecognizedId.getNumericId(),
                pDistanceToRecognizedId, pAngleToRecognizedId);
    }

    // Returns a Pair where first is the inferred distance to the target AprilTag and
    // second is the inferred angle to the target AprilTag.
    public static Pair<Double, Double> calculator(int targetAprilTag, int givenAprilTag, double distanceGivenApril, double angleGivenApril) {

        int sign = 1;
        int endsign = 1;

        //If the given AprilTag is left of the camera, the given AprilTag angle is returned as negative.
        //As a result, we need to compensate by making it positive so the calculations can calculate the Angles and Sides of the right triangles.
        if (targetAprilTag > givenAprilTag) {
            sign = -1;
        }

        //It will give it a positive angle during the calculations if the target AprilTag is left of the given AprilTag as a result of unit circle properties because sine and cosine are both
        //positive in quadrant 1 angles. However, since the AprilTag works with negative angles, we must make it negative.
        if (targetAprilTag < givenAprilTag) {
            endsign = -1;
        }

        //Math shown in the notebook

        angleGivenApril = Math.toRadians(angleGivenApril);
        double c = distanceGivenApril * Math.sin(angleGivenApril);
        double f = (Math.abs(targetAprilTag - givenAprilTag) * 6) - c;
        double d = distanceGivenApril * Math.cos(angleGivenApril);

        double newAngleApril = Math.atan(f / d);
        newAngleApril = endsign * Math.toDegrees(newAngleApril);
        double distanceFive = Math.sqrt(d * d + f * f);

        return Pair.create(distanceFive, newAngleApril);
    }

}
