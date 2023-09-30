package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.opencv.core.Rect;

import java.util.EnumMap;

// Input parameters to barcode recognition.
public class TeamPropParameters {

    // Parameters for the grayscale recognition path.
    public final RedChannelCirclesParameters redChannelCirclesParameters;

    private EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows =
            new EnumMap<>(RobotConstantsCenterStage.SpikeLocationWindow.class);

    public TeamPropParameters(RedChannelCirclesParameters pRedChannelCirclesParameters) {
        redChannelCirclesParameters = pRedChannelCirclesParameters;
    }

    public void setSpikeWindows(EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> pSpikeWindows) {
        spikeWindows = pSpikeWindows;
    }

    public EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> getSpikeWindows() {
        return spikeWindows;
    }

    public static class RedChannelCirclesParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final HoughCirclesFunctionCallParameters houghCirclesFunctionCallParameters;
        public final int maxCircles;

        public RedChannelCirclesParameters(VisionParameters.GrayParameters pGrayParameters,
                                           HoughCirclesFunctionCallParameters pHoughCirclesFunctionCallParameters,
                                           int pMaxCircles) {
         grayParameters = pGrayParameters;
         houghCirclesFunctionCallParameters = pHoughCirclesFunctionCallParameters;
         maxCircles = pMaxCircles;
        }
    }

    // Follows the (not very clear) naming conventions of the OpenCV c++ documentation.
    public static class HoughCirclesFunctionCallParameters {
        public final double dp;
        public final double minDist;
        public final double param1;
        public final double param2;
        public final int minRadius;
        public final int maxRadius;

        public HoughCirclesFunctionCallParameters(double pDp, double pMinDist,
                                                  double pParam1, double pParam2,
                                                  int pMinRadius, int pMaxRadius) {
            dp = pDp;
            minDist = pMinDist;
            param1 = pParam1;
            param2 = pParam2;
            minRadius = pMinRadius;
            maxRadius = pMaxRadius;
        }
    }

}