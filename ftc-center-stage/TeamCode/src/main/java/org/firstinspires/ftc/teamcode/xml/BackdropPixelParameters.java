package org.firstinspires.ftc.teamcode.xml;

// Input parameters to backdrop pixel recognition.
public class BackdropPixelParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;

    public final Criteria aprilTagRectangleCriteria;
    public final Criteria yellowPixelCriteria;

    public BackdropPixelParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                                   Criteria pAprilTagRectangleCriteria,
                                   Criteria pYellowPixelCriteria) {
        grayscaleParameters = pGrayscaleParameters;
        aprilTagRectangleCriteria = pAprilTagRectangleCriteria;
        yellowPixelCriteria = pYellowPixelCriteria;
    }

    public static class Criteria {
        public final double minArea;
        public final double maxArea;
        public final double minAspectRatio;
        public final double maxAspectRatio;

        public Criteria(double pMinArea, double pMaxArea, double pMinAspectRatio, double pMaxAspectRatio) {
            minArea = pMinArea;
            maxArea = pMaxArea;
            minAspectRatio = pMinAspectRatio;
            maxAspectRatio = pMaxAspectRatio;
        }
    }

}