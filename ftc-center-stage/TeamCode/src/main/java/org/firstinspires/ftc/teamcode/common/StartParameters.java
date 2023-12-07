package org.firstinspires.ftc.teamcode.common;

import java.util.EnumMap;

// Starting parameters that can be set via a dedicated TeleOp
// OpMode and read by either an Autonomous or TeleOp competition
// OpMode.
public class StartParameters {

    public final String robotConfigFilename;
    public final String robotActionFilename;
    public final int autoStartDelay;
    public final EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;

    public StartParameters(String pRobotConfigFilename, String pRobotActionFilename, int pAutoStartDelay,
                           EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> pAutoEndingPositions) {
        robotConfigFilename = pRobotConfigFilename;
        robotActionFilename = pRobotActionFilename;
        autoStartDelay = pAutoStartDelay;
        autoEndingPositions = pAutoEndingPositions;
    }

}