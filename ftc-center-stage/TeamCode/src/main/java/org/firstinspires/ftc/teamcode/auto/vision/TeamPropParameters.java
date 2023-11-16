package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to Team Prop recognition.
public class TeamPropParameters {

    public final ColorChannelCirclesParameters colorChannelCirclesParameters;
    public final ColorChannelContoursParameters colorChannelContoursParameters;
    public final ColorChannelPixelCountParameters colorChannelPixelCountParameters;
    public final BrightSpotParameters brightSpotParameters;

    public TeamPropParameters(ColorChannelCirclesParameters pColorChannelCirclesParameters,
                              ColorChannelContoursParameters pColorChannelContoursParameters,
                              ColorChannelPixelCountParameters pColorChannelPixelCountParameters,
                              BrightSpotParameters pBrightSpotParameters
                              ) {
        colorChannelCirclesParameters = pColorChannelCirclesParameters;
        colorChannelContoursParameters = pColorChannelContoursParameters;
        colorChannelPixelCountParameters = pColorChannelPixelCountParameters;
        brightSpotParameters = pBrightSpotParameters;
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

    public static class ColorChannelFeaturesParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final int maxCorners;
        public final double qualityLevel;

        public ColorChannelFeaturesParameters(VisionParameters.GrayParameters pGrayParameters,
                                              int pMaxCorners, double pQualityLevel) {
            grayParameters = pGrayParameters;
            maxCorners = pMaxCorners;
            qualityLevel = pQualityLevel;
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

    public static class ColorChannelPixelCountParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final int minWhitePixelCount;

        public ColorChannelPixelCountParameters(VisionParameters.GrayParameters pGrayParameters, int pMinWhitePixelCount) {
            grayParameters = pGrayParameters;
            minWhitePixelCount = pMinWhitePixelCount;
        }
    }

    public static class BrightSpotParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final double blurKernel;

        public BrightSpotParameters(VisionParameters.GrayParameters pGrayParameters, double pBlurKernel) {
            grayParameters = pGrayParameters;
            blurKernel = pBlurKernel;
        }
    }

}