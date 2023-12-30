package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainConstants;

public class BackdropParameters {

    public final DriveTrainConstants.Direction direction;
    public final double strafeAdjustmentPercent;
    public final double distanceAdjustmentPercent;
    public final double outsideStrafeAdjustment;

    public BackdropParameters(DriveTrainConstants.Direction pDirection,
                              double pStrafeAdjustmentPercent,
                              double pDistanceAdjustmentPercent,
                              double pOutsideStrafeAdjustment) {
        direction = pDirection;
        strafeAdjustmentPercent = pStrafeAdjustmentPercent;
        distanceAdjustmentPercent = pDistanceAdjustmentPercent;
        outsideStrafeAdjustment = pOutsideStrafeAdjustment;
    }
}
