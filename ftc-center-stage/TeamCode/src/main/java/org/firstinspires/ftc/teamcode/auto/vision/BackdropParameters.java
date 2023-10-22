package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainConstants;

public class BackdropParameters {

    public final DriveTrainConstants.Direction direction;
    public final double distanceCameraLensToRobotCenter;
    public final double offsetCameraLensFromRobotCenter;
    public final double strafeAdjustmentPercent;
    public final double distanceAdjustmentPercent;

    public BackdropParameters(DriveTrainConstants.Direction pDirection,
                              double pDistanceCameraLensToRobotCenter,
                              double pOffsetCameraLensFromRobotCenter,
                              double pStrafeAdjustmentPercent,
                              double pDistanceAdjustmentPercent) {
        direction = pDirection;
        distanceCameraLensToRobotCenter = pDistanceCameraLensToRobotCenter;
        offsetCameraLensFromRobotCenter = pOffsetCameraLensFromRobotCenter;
        strafeAdjustmentPercent = pStrafeAdjustmentPercent;
        distanceAdjustmentPercent = pDistanceAdjustmentPercent;
    }
}
