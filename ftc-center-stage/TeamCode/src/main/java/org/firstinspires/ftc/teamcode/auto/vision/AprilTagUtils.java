package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.EnumSet;

public class AprilTagUtils {

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

    //**TODO Changed the order of the parameters and the return value
    // for compatibility with other methods.

    public static Pair<Double, Double> inferAprilTag(RobotConstantsCenterStage.AprilTagId pTargetId,
                                                     RobotConstantsCenterStage.AprilTagId pRecognizedId,
                                                     double pDistanceToRecognizedId, double pAngleToRecognizedId) {
        if (pTargetId == pRecognizedId) // the caller shouldn't do this
            return Pair.create(pDistanceToRecognizedId, pAngleToRecognizedId); // so return the input

        //**TODO Validate that the target id and the recognized id belong to the
        // backdrop of the same alliance.

        //**TODO insert math here ...

        return Pair.create(pDistanceToRecognizedId, pAngleToRecognizedId); //**TODO temp
    }
}
