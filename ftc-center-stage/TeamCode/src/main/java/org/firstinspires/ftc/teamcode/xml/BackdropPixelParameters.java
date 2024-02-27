package org.firstinspires.ftc.teamcode.xml;

// Input parameters to backdrop pixel recognition.
public class BackdropPixelParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;

    public final double aprilTagEdgeMaxAspectRatio;
    public final BoundingBoxCriteria aprilTagBoundingBoxCriteria;
    public final BoundingBoxCriteria yellowPixelBoundingBoxCriteria;

    public BackdropPixelParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                                   double pAprilTagEdgeMaxAspectRatio,
                                   BoundingBoxCriteria pAprilTagCriteria,
                                   BoundingBoxCriteria pYellowPixelCriteria) {
        grayscaleParameters = pGrayscaleParameters;
        aprilTagEdgeMaxAspectRatio = pAprilTagEdgeMaxAspectRatio;
        aprilTagBoundingBoxCriteria = pAprilTagCriteria;
        yellowPixelBoundingBoxCriteria = pYellowPixelCriteria;
    }

    public static class BoundingBoxCriteria {
        public final double minBoundingBoxArea;
        public final double maxBoundingBoxArea;

        public BoundingBoxCriteria(double pMin, double pMax) {
            minBoundingBoxArea = pMin;
            maxBoundingBoxArea = pMax;
        }
    }

}