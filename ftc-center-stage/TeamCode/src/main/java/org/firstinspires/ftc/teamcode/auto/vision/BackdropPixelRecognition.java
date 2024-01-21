package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Objects;

// Determine which of the two slots above the target AprilTag
// the robot should target to deliver a yellow pixel in Autonomous.
// Assume that the slots contain either zero or one pixels.
public class BackdropPixelRecognition {

    private static final String TAG = BackdropPixelRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;

    public BackdropPixelRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
        alliance = pAlliance;
    }

    // Returns the result of image analysis.
    public BackdropPixelReturn recognizePixelsOnBackdropAutonomous(ImageProvider pImageProvider,
                                                                   VisionParameters.ImageParameters pImageParameters,
                                                                   BackdropPixelParameters pBackdropPixelParameters,
                                                                   RobotConstantsCenterStage.BackdropPixelRecognitionPath pBackdropPixelRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In BackdropPixelRecognition.recognizePixelsOnBackdropAutonomous");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> backdropPixelImage = pImageProvider.getImage();
        if (backdropPixelImage == null)
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getDateTimeStamp(backdropPixelImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(backdropPixelImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pBackdropPixelRecognitionPath);
        switch (pBackdropPixelRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE: {
                return redChannelPathWebcam(pImageParameters, imageROI, outputFilenamePreamble, pBackdropPixelParameters);
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private BackdropPixelReturn redChannelPathWebcam(VisionParameters.ImageParameters pImageParameters, Mat pImageROI, String pOutputFilenamePreamble,
                                                     BackdropPixelParameters pBackdropPixelParameters) {

        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pBackdropPixelParameters.grayscaleParameters.median_target, pBackdropPixelParameters.grayscaleParameters.threshold_low);

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Within the ROI draw all of the contours.
        RobotLogCommon.d(TAG, "Number of contours " + contours.size());
        if (pOutputFilenamePreamble != null && RobotLogCommon.isLoggable("v")) {
            Mat contoursDrawn = pImageROI.clone();
            ShapeDrawing.drawShapeContours(contours, contoursDrawn);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursDrawn);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");
        }

        // AN AprilTag should be recognizable as a rectangle.
        // The pixel should be recognizable as a hexagon.
        // See https://www.programcreek.com/java-api-examples/?class=org.opencv.imgproc.Imgproc&method=arcLength
        int imageCenterX = pImageParameters.resolution_width / 2;
        RobotLogCommon.d(TAG, "Image center " + imageCenterX);

        boolean foundAnAprilTag = false;
        Rect mostCentralAprilTagRect = null;
        int mostCentralAprilTagCenterX = 1000; // impossibly high
        boolean foundAPixel = false;
        Rect validatedPixelRect = null;
        int validatedPixelCenterX = 0;

        Mat drawnRectangle = pImageROI.clone();
        MatOfPoint2f contourPoints2f;
        for (MatOfPoint oneContour : contours) {
            contourPoints2f = new MatOfPoint2f(oneContour.toArray()); // Point2f for approxPolyDP
            double perimeter = Imgproc.arcLength(contourPoints2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourPoints2f, approx, 0.03 * perimeter, true); // simplify the contour
            Point[] points = approx.toArray();

            if (points.length == 4) {
                RobotLogCommon.d(TAG, "Found a polygon with 4 sides");
                Rect aprilTagBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "Area of a possible AprilTag's contour bounding box: " + aprilTagBoundingRect.area());

                //&& Could filter the AprilTag bounding box shape also - close to square.
                if (aprilTagBoundingRect.area() < pBackdropPixelParameters.aprilTagBoundingBoxCriteria.minBoundingBoxArea ||
                        aprilTagBoundingRect.area() > pBackdropPixelParameters.aprilTagBoundingBoxCriteria.maxBoundingBoxArea) {
                    RobotLogCommon.d(TAG, "The possible AprilTag violates the size criteria");
                    continue;
                }

                // Save the AprilTag bounding rectangle whose center is closest
                // to the center of the full image.
                int centerOfAprilTagBoundingBoxX = pImageParameters.image_roi.x + aprilTagBoundingRect.x + (aprilTagBoundingRect.width / 2);
                RobotLogCommon.d(TAG, "Center of AprilTag bounding box in full image: x " + centerOfAprilTagBoundingBoxX +
                        ", y " + (pImageParameters.image_roi.y + aprilTagBoundingRect.y + (aprilTagBoundingRect.height / 2)));

                foundAnAprilTag = true;
                RobotLogCommon.d(TAG, "Found an AprilTag on the backdrop");

                // Save this AprilTag bounding box if it's center is closest to the
                // center of the ROI.
                if (Math.abs(imageCenterX - centerOfAprilTagBoundingBoxX) < mostCentralAprilTagCenterX) {
                    mostCentralAprilTagCenterX = centerOfAprilTagBoundingBoxX;
                    mostCentralAprilTagRect = aprilTagBoundingRect;
                }

                continue;
            }

            if (points.length == 6) {
                if (foundAPixel) // only need one
                    continue;

                RobotLogCommon.d(TAG, "Found a polygon with 6 sides");
                Rect pixelBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "Area of a possible pixel's hexagonal contour bounding box: " + pixelBoundingRect.area());

                // Rule out blobs that may qualify as hexagons.
                //&& Could filter the yellow pixel bounding box shape also - close to square.
                if (pixelBoundingRect.area() < pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.minBoundingBoxArea ||
                        pixelBoundingRect.area() > pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.maxBoundingBoxArea) {
                    RobotLogCommon.d(TAG, "The possible pixel violates the size criteria");
                    continue;
                }

                foundAPixel = true;
                validatedPixelRect = pixelBoundingRect;
                validatedPixelCenterX = pImageParameters.image_roi.x + validatedPixelRect.x + (validatedPixelRect.width / 2);
                RobotLogCommon.d(TAG, "Found a yellow pixel on the backdrop");
            }
        }

        // This is the most desirable case.
        if (foundAnAprilTag && foundAPixel) {
            // Draw a rectangle around the most central AprilTag and around the pixel hexagon.
            ShapeDrawing.drawOneRectangle(mostCentralAprilTagRect, drawnRectangle, 2);
            ShapeDrawing.drawOneRectangle(validatedPixelRect, drawnRectangle, 2);

            // Compare the relative positions of the center of the AprilTag closest
            // to the center of the full image and the center of the yellow pixel.
            RobotLogCommon.d(TAG, "Most central AprilTag center x " + mostCentralAprilTagCenterX);
            RobotLogCommon.d(TAG, "Most central AprilTag bounding box width " + Objects.requireNonNull(mostCentralAprilTagRect, TAG + "mostCentralAprilTagRect is null").width);

            RobotLogCommon.d(TAG, "Yellow pixel bounding box center x " + validatedPixelCenterX +
                    ", y " + (validatedPixelRect.y + (validatedPixelRect.height / 2)));

            // Need a more sophisticated comparison to filter out the case
            // where our alliance partner misplaced the pixel to the left or right
            // of the two target locations. We can't use absolute pixel counts but
            // a percentage should work, e.g. if the center of the pixel is at 235
            // x in the full
            // image and the center of the AprilTag is at 330 x then the pixel position is .71
            // of that of the AprilTag and is misplaced to the left. If the pixel
            // position is at 420 x then it is 1.27 more than the AprilTag and is
            // misplaced to the right.
            double PIXEL_OUT_OF_RAGE_LOW = .71;
            double PIXEL_OUT_OF_RANGE_HIGH= 1.27;
            RobotConstantsCenterStage.BackdropPixelOpenSlot backdropPixelOpenSlot;
            if ((validatedPixelCenterX / (double) mostCentralAprilTagCenterX) <= PIXEL_OUT_OF_RAGE_LOW) {
                backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT;
                RobotLogCommon.d(TAG, "The yellow pixel was misdelivered to the left");
            } else if ((validatedPixelCenterX / (double) mostCentralAprilTagCenterX) >= PIXEL_OUT_OF_RANGE_HIGH) {
                backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT;
                RobotLogCommon.d(TAG, "The yellow pixel was misdelivered to the left");
            } else if (validatedPixelCenterX == mostCentralAprilTagCenterX) {
                backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT;
                RobotLogCommon.d(TAG, "The yellow pixel is exactly above the AprilTag");
            } else if (validatedPixelCenterX > mostCentralAprilTagCenterX) {
                backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.LEFT;
                RobotLogCommon.d(TAG, "The open slot is on the left");
            } else {
                backdropPixelOpenSlot = RobotConstantsCenterStage.BackdropPixelOpenSlot.RIGHT;
                RobotLogCommon.d(TAG, "The open slot is on the right");
            }

            if (RobotLogCommon.isLoggable("v")) {
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRECT.png", drawnRectangle);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");
            }

            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    backdropPixelOpenSlot);
        }

        if (foundAnAprilTag) {
            // Not so bad: there are no pixels already on the backdrop.
            ShapeDrawing.drawOneRectangle(mostCentralAprilTagRect, drawnRectangle, 2);
            RobotLogCommon.d(TAG, "Did not find a pixel (hexagon); both slots are open");
        } else {
            // Not good at all: only found a pixel but I don't know where it is.
            ShapeDrawing.drawOneRectangle(validatedPixelRect, drawnRectangle, 2);
            RobotLogCommon.d(TAG, "Found a pixel but not an AprilTag");
        }

        if (RobotLogCommon.isLoggable("v")) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRECT.png", drawnRectangle);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");
        }

        return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
    }
}