package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to backdrop pixel recognition.
public class BackdropPixelParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;

    public final BoundingBoxCriteria aprilTagBoundingBoxCriteria;
    public final BoundingBoxCriteria yellowPixelBoundingBoxCriteria;

    public BackdropPixelParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                                   BoundingBoxCriteria pAprilTagCriteria,
                                   BoundingBoxCriteria pYellowPixelCriteria) {
        grayscaleParameters = pGrayscaleParameters;
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