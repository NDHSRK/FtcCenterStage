package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to Team Prop recognition.
public class TeamPropParameters {

    public final ColorChannelCirclesParameters colorChannelCirclesParameters;
    public final ColorChannelPixelCountParameters colorChannelPixelCountParameters;
    public final BrightSpotParameters brightSpotParameters;

    public TeamPropParameters(ColorChannelCirclesParameters pColorChannelCirclesParameters,
                              ColorChannelPixelCountParameters pColorChannelPixelCountParameters,
                              BrightSpotParameters pBrightSpotParameters
    ) {
        colorChannelCirclesParameters = pColorChannelCirclesParameters;
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

    //## Not supported in Android - see IJCenterStageVision.
    // public static class ColorChannelFeaturesParameters {
    // public static class ColorChannelContoursParameters {

    public static class ColorChannelPixelCountParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final int redMinWhitePixelCount;
        public final VisionParameters.GrayParameters blueGrayParameters;
        public final int blueMinWhitePixelCount;

        public ColorChannelPixelCountParameters(VisionParameters.GrayParameters pRedGrayParameters, int pRedMinWhitePixelCount,
                                                VisionParameters.GrayParameters pBlueGrayParameters, int pBlueMinWhitePixelCount) {
            redGrayParameters = pRedGrayParameters;
            redMinWhitePixelCount = pRedMinWhitePixelCount;
            blueGrayParameters = pBlueGrayParameters;
            blueMinWhitePixelCount = pBlueMinWhitePixelCount;
        }
    }

    public static class BrightSpotParameters {
        public final VisionParameters.GrayParameters redGrayParameters;
        public final double redBlurKernel;
        public final VisionParameters.GrayParameters blueGrayParameters;
        public final double blueBlurKernel;

        public BrightSpotParameters(VisionParameters.GrayParameters pRedGrayParameters, double pRedBlurKernel,
                                    VisionParameters.GrayParameters pBlueGrayParameters, double pBlueBlurKernel) {
            redGrayParameters = pRedGrayParameters;
            redBlurKernel = pRedBlurKernel;
            blueGrayParameters = pBlueGrayParameters;
            blueBlurKernel = pBlueBlurKernel;
        }
    }

}