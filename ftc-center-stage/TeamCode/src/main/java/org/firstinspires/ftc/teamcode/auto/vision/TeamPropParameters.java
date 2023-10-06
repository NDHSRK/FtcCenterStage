package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.opencv.core.Rect;

import java.util.EnumMap;

// Input parameters to barcode recognition.
public class TeamPropParameters {

    public final ColorChannelCirclesParameters colorChannelCirclesParameters;
    public final ColorChannelBrightSpotParameters colorChannelBrightSpotParameters;
    public final ColorChannelContoursParameters colorChannelContoursParameters;

    private EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows =
            new EnumMap<>(RobotConstantsCenterStage.SpikeLocationWindow.class);

    public TeamPropParameters(ColorChannelCirclesParameters pColorChannelCirclesParameters,
                              ColorChannelBrightSpotParameters pColorChannelBrightSpotParameters,
                              ColorChannelContoursParameters pColorChannelContoursParameters) {
        colorChannelCirclesParameters = pColorChannelCirclesParameters;
        colorChannelBrightSpotParameters = pColorChannelBrightSpotParameters;
        colorChannelContoursParameters = pColorChannelContoursParameters;
    }

    public void setSpikeWindows(EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> pSpikeWindows) {
        spikeWindows = pSpikeWindows;
    }

    public EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> getSpikeWindows() {
        return spikeWindows;
    }

    public static class ColorChannelCirclesParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final HoughCirclesFunctionCallParameters houghCirclesFunctionCallParameters;
        public final int maxCircles;

        public ColorChannelCirclesParameters(VisionParameters.GrayParameters pGrayParameters,
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

    public static class ColorChannelBrightSpotParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final double blurKernel;

        public ColorChannelBrightSpotParameters(VisionParameters.GrayParameters pGrayParameters, double pBlurKernel) {
            grayParameters = pGrayParameters;
            blurKernel = pBlurKernel;
        }
    }

    public static class ColorChannelContoursParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final double minArea;
        public final double maxArea;

        public ColorChannelContoursParameters(VisionParameters.GrayParameters pGrayParameters,
                                              double pMinArea, double pMaxArea) {
            grayParameters = pGrayParameters;
            minArea = pMinArea;
            maxArea = pMaxArea;
        }
    }

}