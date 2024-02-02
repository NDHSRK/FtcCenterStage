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

// Determine which of the two slots above the target AprilTag
// the robot should target to deliver a yellow pixel in Autonomous.
// Assume that the slots contain either zero or one pixels.
public class BackdropPixelRecognition {

    private static final String TAG = BackdropPixelRecognition.class.getSimpleName();
    private static final double PIXEL_OUT_OF_RAGE_LOW = .71;
    private static final double PIXEL_OUT_OF_RANGE_HIGH = 1.27;

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

        // The image is in BGR order.
        String fileDate = TimeStamp.getDateTimeStamp(backdropPixelImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(backdropPixelImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pBackdropPixelRecognitionPath);
        if (pBackdropPixelRecognitionPath != RobotConstantsCenterStage.BackdropPixelRecognitionPath.RED_CHANNEL_GRAYSCALE)
            throw new AutonomousRobotException(TAG, "Unrecognized recognition path");

        return redChannelPathWebcam(pImageParameters, imageROI, outputFilenamePreamble, pBackdropPixelParameters);
    }

    private BackdropPixelReturn redChannelPathWebcam(VisionParameters.ImageParameters pImageParameters, Mat pImageROI, String pOutputFilenamePreamble,
                                                     BackdropPixelParameters pBackdropPixelParameters) {

        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pBackdropPixelParameters.grayscaleParameters.median_target, pBackdropPixelParameters.grayscaleParameters.threshold_low);

        //## Canny produces a very clean image but findContours works fine
        // on the thresholded image.
        //Mat canny = new Mat();
        //Imgproc.Canny(thresholded, canny, 150, 250);
        //Imgcodecs.imwrite(pOutputFilenamePreamble + "_CANNY.png", canny);

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

        // An AprilTag should be recognizable as a rectangle. Find the rectangle
        // closest to the center of the image.
        // See https://www.programcreek.com/java-api-examples/?class=org.opencv.imgproc.Imgproc&method=arcLength
        Mat drawnBackdropObjects = pImageROI.clone();
        MatOfPoint2f contourPoints2f;
        double perimeter;
        Point[] points;
        List<MatOfPoint> pixelContourCandidates = new ArrayList<>();

        int imageCenterX = pImageParameters.resolution_width / 2;
        boolean foundAnAprilTag = false;
        Rect mostCentralAprilTagRect = null;
        int mostCentralAprilTagCenterX = 1000; // impossibly high
        for (MatOfPoint oneContour : contours) {
            contourPoints2f = new MatOfPoint2f(oneContour.toArray()); // Point2f for approxPolyDP
            perimeter = Imgproc.arcLength(contourPoints2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourPoints2f, approx, 0.03 * perimeter, true); // simplify the contour
            points = approx.toArray();

            if (points.length == 4) {
                RobotLogCommon.d(TAG, "Found a polygon with 4 sides");
                Rect aprilTagBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "Area of a possible AprilTag's contour bounding box: " + aprilTagBoundingRect.area());

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
            } else {
                // Found a contour that is not an AprilTag. Save it for
                // pixel detection.
                pixelContourCandidates.add(oneContour);
                RobotLogCommon.d(TAG, "Found a contour that does not have 4 sides: centroid " + ImageUtils.getContourCentroid(oneContour) + ", contour area " + Imgproc.contourArea(oneContour));
            }
        }

        // If we haven't found an AprilTag then there's no reason to look
        // for a pixel.
        if (!foundAnAprilTag) {
            RobotLogCommon.d(TAG, "Did not find any AprilTags");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        ShapeDrawing.drawOneRectangle(mostCentralAprilTagRect, drawnBackdropObjects, 2);

        RobotLogCommon.d(TAG, "Look for a hexagon that encloses a yellow pixel");
        List<MatOfPoint> disjointedPixelContourCandidates = new ArrayList<>();
        boolean foundAPixel = false;
        Rect validatedPixelRect = null;
        int validatedPixelCenterX = 0;
        for (MatOfPoint oneContour : pixelContourCandidates) {
            contourPoints2f = new MatOfPoint2f(oneContour.toArray()); // Point2f for approxPolyDP
            perimeter = Imgproc.arcLength(contourPoints2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourPoints2f, approx, 0.03 * perimeter, true); // simplify the contour
            points = approx.toArray();

            if (points.length == 6) {
                RobotLogCommon.d(TAG, "Found a polygon with 6 sides");
                Rect pixelBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "Area of a possible pixel's hexagonal contour bounding box: " + pixelBoundingRect.area());

                // Rule out small blobs that have qualified as hexagons.
                if (pixelBoundingRect.area() < pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.minBoundingBoxArea ||
                        pixelBoundingRect.area() > pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.maxBoundingBoxArea) {
                    disjointedPixelContourCandidates.add(oneContour); // include as a disjointed contour
                    RobotLogCommon.d(TAG, "The possible pixel violates the size criteria");
                    continue;
                }

                // Qualify the left and right boundaries of the pixel's bounding box
                // with respect to the AprilTag's center.
                if (qualifyPixel(pImageParameters, mostCentralAprilTagCenterX, pixelBoundingRect)) {
                    foundAPixel = true;
                    validatedPixelRect = pixelBoundingRect;
                    validatedPixelCenterX = pImageParameters.image_roi.x + validatedPixelRect.x + (validatedPixelRect.width / 2);
                    ShapeDrawing.drawOneRectangle(validatedPixelRect, drawnBackdropObjects, 2);
                    RobotLogCommon.d(TAG, "Found a yellow pixel on the backdrop");
                    break; // only need one pixel
                }
            } else {
                // Didn't find a hexagon but there may be a cluster of contours
                // that we can take as a pixel.
                disjointedPixelContourCandidates.add(oneContour);
            }
        }

        // If we haven't found a hexagon then see if there's a
        // cluster of contours that we can assume is a pixel.
        int qualifyingDisjointedCountourCenters = 0;
        int accumQualifyingCentersX = 0;
        int averageQualifyingPixelCenterX;
        int contourBoundingBoxLowestX = 1000, contourBoundingBoxHighestX = 0;
        int contourBoundingBoxLowestY = 1000, contourBoundingBoxHighestY = 0;
        if (!foundAPixel) {
            RobotLogCommon.d(TAG, "Did not find a complete hexagon; try to find a cluster of disjoined contours that we can assume is a pixel");
            RobotLogCommon.d(TAG, "Number of disjointed contours " + disjointedPixelContourCandidates.size());

            // Qualify disjointed contours for pixel detection according to the
            // horizontal position of their bounding boxes.
            Rect contourBoundingRect;
            int boundingBoxCenterX;
            for (MatOfPoint oneContour : disjointedPixelContourCandidates) {
                contourBoundingRect = Imgproc.boundingRect(oneContour);
                boundingBoxCenterX = pImageParameters.image_roi.x + contourBoundingRect.x +
                        (contourBoundingRect.width / 2);

                // If the right boundary of the contour's bounding box is to the
                // left of the two slots that belong to the AprilTag then disqualify
                // this contour. Also disqualify this contour if the left boundary
                // of the contour's bounding box is to the right of the two slots
                // that belong to the AprilTag.
                if (!qualifyPixel(pImageParameters, mostCentralAprilTagCenterX, contourBoundingRect)) {
                    RobotLogCommon.d(TAG, "Possible pixel contour is *not* within range of the AprilTag");
                    continue;
                }

                qualifyingDisjointedCountourCenters++;
                accumQualifyingCentersX += boundingBoxCenterX;
                RobotLogCommon.d(TAG, "Possible pixel contour *is* within range of the AprilTag");

                // Pay attention to each bounding box because at the end we want to draw an enclosing
                // rectangle around all of the disjoint contours.
                if (contourBoundingRect.x < contourBoundingBoxLowestX)
                    contourBoundingBoxLowestX = contourBoundingRect.x;

                if (contourBoundingRect.x + contourBoundingRect.width > contourBoundingBoxHighestX)
                    contourBoundingBoxHighestX = contourBoundingRect.x + contourBoundingRect.width;

                if (contourBoundingRect.y < contourBoundingBoxLowestY)
                    contourBoundingBoxLowestY = contourBoundingRect.y;

                if (contourBoundingRect.y + contourBoundingRect.height > contourBoundingBoxHighestY)
                    contourBoundingBoxHighestY = contourBoundingRect.y + contourBoundingRect.height;
             }
        }

        // We have found an AprilTag. If we haven't found a pixel -
        // either a hexagon or a cluster of disjointed contours - then
        // we're done.
        if (!foundAPixel && qualifyingDisjointedCountourCenters == 0) {
            RobotLogCommon.d(TAG, "Did not find a pixel (hexagon); both slots are open");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // This is the most desirable case: an AprilTag and a pixel hexagon.
        RobotLogCommon.d(TAG, "Most central AprilTag center x " + mostCentralAprilTagCenterX);
        RobotConstantsCenterStage.BackdropPixelOpenSlot backdropPixelOpenSlot;
        if (foundAPixel) {
            // Compare the relative positions of the center of the AprilTag closest
            // to the center of the full image and the center of the yellow pixel.
            RobotLogCommon.d(TAG, "Yellow pixel bounding box center x " + validatedPixelCenterX +
                    ", y " + (validatedPixelRect.y + (validatedPixelRect.height / 2)));

            backdropPixelOpenSlot = determinePixelOpenSlot(mostCentralAprilTagCenterX, validatedPixelCenterX);
        } else { // There must be a cluster of contours that comprises a pixel.
            Rect enclosingBox = new Rect(contourBoundingBoxLowestX, contourBoundingBoxLowestY,
                    contourBoundingBoxHighestX - contourBoundingBoxLowestX,
                    contourBoundingBoxHighestY - contourBoundingBoxLowestY);
            ShapeDrawing.drawOneRectangle(enclosingBox, drawnBackdropObjects, 2);
            RobotLogCommon.d(TAG, "Draw a rectangle around the cluster of pixels " + enclosingBox);

            averageQualifyingPixelCenterX = accumQualifyingCentersX / qualifyingDisjointedCountourCenters;
            RobotLogCommon.d(TAG, "Average center of disjointed pixel contours " + averageQualifyingPixelCenterX);
            backdropPixelOpenSlot = determinePixelOpenSlot(mostCentralAprilTagCenterX, averageQualifyingPixelCenterX);
        }

        if (RobotLogCommon.isLoggable("v")) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRECT.png", drawnBackdropObjects);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");
        }

        return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, backdropPixelOpenSlot);
    }

    // We need to filter out the case where our alliance partner misplaced
    // the pixel to the left or right of the two target locations, one on
    // each side of the AprilTag. We can't use absolute pixel counts but
    // a percentage should work, e.g. if the right edge of the pixel is at
    // 235 x in the full image and the center of the AprilTag is at 330 x
    // then the pixel position is .71 of that of the AprilTag and is misplaced
    // to the left. If the left edge of the pixel is at 420 x then it is 1.27
    // more than the AprilTag and is misplaced to the right.
    private boolean qualifyPixel(VisionParameters.ImageParameters pImageParameters, int pMostCentralAprilTagX, Rect pPixelContourBoundingBox) {
        // If the right boundary of the contour's bounding box is to the
        // left of the two slots that belong to the AprilTag then disqualify
        // this contour.
        int boundingBoxLeftBoundary = pImageParameters.image_roi.x + pPixelContourBoundingBox.x;
        int boundingBoxRightBoundary = pImageParameters.image_roi.x + pPixelContourBoundingBox.x + pPixelContourBoundingBox.width;
        int boundingBoxTopBoundary = pImageParameters.image_roi.y + pPixelContourBoundingBox.y;
        int boundingBoxBottomBoundary = pImageParameters.image_roi.y + pPixelContourBoundingBox.y + pPixelContourBoundingBox.height;
        RobotLogCommon.d(TAG, "Disjointed pixel contour's bounding box area: " + pPixelContourBoundingBox.area() +
                ", low x " + boundingBoxLeftBoundary + ", high x " + boundingBoxRightBoundary +
                ", low y " + boundingBoxTopBoundary + ", high y " + boundingBoxBottomBoundary);

        if ((boundingBoxRightBoundary / (double) pMostCentralAprilTagX) <= PIXEL_OUT_OF_RAGE_LOW) {
            RobotLogCommon.d(TAG, "Possible pixel contour is too far to the left of the AprilTag");
            return false;
        }

        // If the left boundary of the contour's bounding box is to the
        // right of the two slots that belong to the AprilTag then disqualify
        // this contour.
        if ((boundingBoxLeftBoundary / (double) pMostCentralAprilTagX) >= PIXEL_OUT_OF_RANGE_HIGH) {
            RobotLogCommon.d(TAG, "Possible pixel contour is too far to the right of the AprilTag");
            return false;
        }

        return true;
    }

    private RobotConstantsCenterStage.BackdropPixelOpenSlot
    determinePixelOpenSlot(int pMostCentralAprilTagX, int pCenterOfObjectX) {
        if (pCenterOfObjectX == pMostCentralAprilTagX) {
            RobotLogCommon.d(TAG, "The yellow pixel is exactly above the AprilTag");
            return RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT;
        }

        if (pCenterOfObjectX > pMostCentralAprilTagX) {
            RobotLogCommon.d(TAG, "The open slot is on the left");
            return RobotConstantsCenterStage.BackdropPixelOpenSlot.LEFT;
        }

        RobotLogCommon.d(TAG, "The open slot is on the right");
        return RobotConstantsCenterStage.BackdropPixelOpenSlot.RIGHT;
    }

}