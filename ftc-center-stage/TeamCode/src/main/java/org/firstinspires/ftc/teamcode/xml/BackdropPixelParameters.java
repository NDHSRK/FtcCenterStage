package org.firstinspires.ftc.teamcode.xml;

// Input parameters to backdrop pixel recognition.
public class BackdropPixelParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;

    public final double aprilTagRectangleMinAspectRatio;
    public final double aprilTagRectangleMaxAspectRatio;
    public final AreaLimits aprilTagRectangleAreaLimits;
    public final AreaLimits yellowPixelAreaLimits;

    public BackdropPixelParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                                   double pAprilTagRectangleMinAspectRatio,
                                   double pAprilTagRectangleMaxAspectRatio,
                                   AreaLimits pAprilTagRectangleCriteria,
                                   AreaLimits pYellowPixelCriteria) {
        grayscaleParameters = pGrayscaleParameters;
        aprilTagRectangleMinAspectRatio = pAprilTagRectangleMinAspectRatio;
        aprilTagRectangleMaxAspectRatio = pAprilTagRectangleMaxAspectRatio;
        aprilTagRectangleAreaLimits = pAprilTagRectangleCriteria;
        yellowPixelAreaLimits = pYellowPixelCriteria;
    }

    public static class AreaLimits {
        public final double minArea;
        public final double maxArea;

        public AreaLimits(double pMin, double pMax) {
            minArea = pMin;
            maxArea = pMax;
        }
    }

}