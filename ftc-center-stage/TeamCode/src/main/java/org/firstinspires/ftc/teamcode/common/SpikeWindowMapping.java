package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.opencv.core.Rect;

import java.util.EnumMap;

public class SpikeWindowMapping {
    public final VisionParameters.ImageParameters imageParameters;
    public final RobotConstantsCenterStage.TeamPropRecognitionPath recognitionPath;
    public final EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows;

    public SpikeWindowMapping(VisionParameters.ImageParameters pImageParameters,
                           RobotConstantsCenterStage.TeamPropRecognitionPath pRecognitionPath,
                           EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> pSpikeWindows) {
        imageParameters = pImageParameters;
        recognitionPath = pRecognitionPath;
        spikeWindows = pSpikeWindows;
    }

}
