package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.firstinspires.ftc.teamcode.xml.BackdropPixelParameters;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Date;
import java.util.Comparator;
import java.util.List;

import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;

// Determine which of the two slots above the target AprilTag
// the robot should target to deliver a yellow pixel in Autonomous.
// Assume that the slots contain either zero or one pixels.
public class BackdropPixelRecognition {

    private static final String TAG = BackdropPixelRecognition.class.getSimpleName();
    private static final double PIXEL_OUT_OF_RAGE_LOW = .71;
    private static final double PIXEL_OUT_OF_RANGE_HIGH = 1.27;

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;

    // BackdropPixelRecognition can write out a lot of intermediate
    // image files. Use this flag to enable or disable the writing
    // of these files. This way you can use the general "d" setting
    // in the logger but temporarily write the image files here.
    private final boolean writeIntermediateImageFiles = true;

    public BackdropPixelRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
        alliance = pAlliance;
    }

    // Returns the result of image analysis.
    public BackdropPixelReturn recognizePixelsOnBackdropAutonomous(ImageProvider pImageProvider,
                                                                   VisionParameters.ImageParameters pImageParameters,
                                                                   BackdropPixelParameters pBackdropPixelParameters,
                                                                   double pAngleFromCameraToAprilTag,
                                                                   double pDistanceFromCameraToAprilTag,
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

        return redChannelPathWebcam(pImageParameters, imageROI, outputFilenamePreamble, pBackdropPixelParameters,
                pAngleFromCameraToAprilTag, pDistanceFromCameraToAprilTag);
    }

    private BackdropPixelReturn redChannelPathWebcam(VisionParameters.ImageParameters pImageParameters, Mat pImageROI, String pOutputFilenamePreamble,
                                                     BackdropPixelParameters pBackdropPixelParameters,
                                                     double pAngleFromCameraToAprilTag,
                                                     double pDistanceFromCameraToAprilTag) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pBackdropPixelParameters.grayscaleParameters.median_target, pBackdropPixelParameters.grayscaleParameters.threshold_low);

        // Look for one or more vertical lines that are the edges of AprilTags.
        // From https://stackoverflow.com/questions/60521925/how-to-detect-the-horizontal-and-vertical-lines-of-a-table-and-eliminate-the-noi
        Mat openedVertical = new Mat();
        Mat vertical_kernel = Imgproc.getStructuringElement(MORPH_RECT, new Size(1, 50));
        Imgproc.morphologyEx(thresholded, openedVertical, MORPH_OPEN, vertical_kernel);

        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_VERT.png", openedVertical);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_VERT.png");
        }

        //## Too many horizontal lines ...
        /*
        Mat openedHorizontal = new Mat();
        Mat horizontal_kernel = Imgproc.getStructuringElement(MORPH_RECT, new Size(50,1));
        Imgproc.morphologyEx(thresholded, openedHorizontal, MORPH_OPEN, horizontal_kernel);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_HORIZ.png", openedHorizontal);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_HORIZ.png");
        */

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(openedVertical, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Within the ROI draw all of the contours.
        RobotLogCommon.d(TAG, "Number of contours " + contours.size());
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Mat contoursDrawn = pImageROI.clone();
            ShapeDrawing.drawShapeContours(contours, contoursDrawn);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_ECON.png", contoursDrawn);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ECON.png");
        }

        // Enclose each contour in a bounding box.
        Mat drawnBoundingBoxes = pImageROI.clone();
        Rect oneBoundingRect;
        ArrayList<Rect> verticalBoundingBoxes = new ArrayList<>();
        for (MatOfPoint oneContour : contours) {
            oneBoundingRect = Imgproc.boundingRect(oneContour);

            RobotLogCommon.d(TAG, "Contour bounding box: width " + oneBoundingRect.width +
                    ", height " + oneBoundingRect.height +
                    ", x " + oneBoundingRect.x +
                    ", y " + oneBoundingRect.y);

            // Check the aspect ratio of what we think is a vertical edge of an AprilTag.
            if ((double) oneBoundingRect.width / (double) oneBoundingRect.height <= pBackdropPixelParameters.aprilTagEdgeMaxAspectRatio) {
                verticalBoundingBoxes.add(oneBoundingRect);
                ShapeDrawing.drawOneRectangle(oneBoundingRect, drawnBoundingBoxes, 2);
            } else RobotLogCommon.d(TAG, "Bounding box over the allowable aspect ratio");
        }

        if (verticalBoundingBoxes.isEmpty()) {
            RobotLogCommon.d(TAG, "Failed to find the edge of an AprilTag");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }
        
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_EDGE.png", drawnBoundingBoxes);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_EDGE.png");
        }

        // Collect potential AprilTag edges by collecting lines according
        // to the proximity of their x-coordinates.
        verticalBoundingBoxes.sort(Comparator.comparingInt((Rect r) -> r.x));
        int currentX = -1;
        int currentWidth = 0;
        ArrayList<Rect> oneVerticalLine = new ArrayList<>();
        ArrayList<ArrayList<Rect>> allVerticalLines = new ArrayList<>();
        for (Rect rect : verticalBoundingBoxes) {
            if (rect.x > currentX + currentWidth) {
                if (!oneVerticalLine.isEmpty()) {
                    allVerticalLines.add(new ArrayList<Rect>(oneVerticalLine)); // need copy
                    oneVerticalLine.clear();
                }

                currentX = rect.x;
                currentWidth = rect.width;
                oneVerticalLine.add(rect);
            }
        }

        // Iterate through the collection of line segments and find (or
        // synthesize) the tallest.
        Rect tallestLine = new Rect(0, 0, 0, 0);
        Rect firstSegment;
        int lowestY;
        Rect highestSegment;
        int highestY;
        int synthesizedLineHeight;
        for (ArrayList<Rect> oneLine : allVerticalLines) {
            for (Rect oneSegment : oneLine) {
                if (oneLine.size() == 1) {
                    if (oneSegment.height > tallestLine.height) {
                        tallestLine = oneSegment;
                    }
                } else { // the line consists of more than one segment
                    // Sort the segments by their y-coordinates.
                    oneLine.sort(Comparator.comparingInt((Rect r) -> r.y));

                    // Synthesize a single line by combining the segments.
                    firstSegment = oneLine.get(0);
                    lowestY = firstSegment.y;
                    highestSegment = oneLine.get(oneLine.size() - 1);
                    highestY = highestSegment.y + highestSegment.height;
                    synthesizedLineHeight = highestY - lowestY;
                    if (synthesizedLineHeight > tallestLine.height) {
                        tallestLine = new Rect(firstSegment.x, firstSegment.y,
                                firstSegment.width, synthesizedLineHeight);
                    }
                }
            }
        }

        // Assume that the tallest line is a vertical edge of an AprilTag.
        // The known height of the AprilTag sticker including its white boundary
        // is 3.0". From this we can calculate pixels/inch. However, because the
        // backdrop is at a 30-degree angle we need to add a factor for "perspective
        // compression" because in the image the edge will appear to be shorter than
        // it really is.
        RobotLogCommon.d(TAG, "The height of the tallest AprilTag edge is " + tallestLine.height);
        RobotLogCommon.d(TAG, "Applying perspective compression factor of 30%");
        double pixelsPerInch = (tallestLine.height + (tallestLine.height * 0.3)) / 3.0;
        RobotLogCommon.d(TAG, "Adjusted backdrop pixels per inch " + pixelsPerInch);

        // We have the angle from the camera to the center of the target
        // AprilTag (left-of-center is a positive angle, right-of-center
        // is negative).
        int signOfAngle = (int) Math.signum(pAngleFromCameraToAprilTag);
        double angleFromCameraToAprilTag = Math.abs(pAngleFromCameraToAprilTag);
        double sinA = Math.sin(Math.toRadians(angleFromCameraToAprilTag));
        double oppositeInches = sinA * pDistanceFromCameraToAprilTag;
        int oppositePixels = (int) (pixelsPerInch * oppositeInches);

        RobotLogCommon.d(TAG, "Distance from image center to AprilTag center " + oppositeInches +
                ", pixels " + oppositePixels);

        // If the original angle was positive then *subtract* else *add*.
        int imageCenter = pImageParameters.resolution_width / 2;
        int targetAprilTagCenterX = (signOfAngle > 0) ? imageCenter - oppositePixels :
                imageCenter + oppositePixels;
        RobotLogCommon.d(TAG, "Center of the target AprilTag in pixels: " + targetAprilTagCenterX);

        RobotLogCommon.d(TAG, "Look for a hexagon that encloses a yellow pixel");
        List<MatOfPoint> pixelContours = new ArrayList<>();
        Imgproc.findContours(thresholded, pixelContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (pixelContours.size() == 0) {
            RobotLogCommon.d(TAG, "No pixel contours found");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Within the ROI draw all of the contours.
        RobotLogCommon.d(TAG, "Number of pixel contours " + pixelContours.size());
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Mat contoursDrawn = pImageROI.clone();
            ShapeDrawing.drawShapeContours(pixelContours, contoursDrawn);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_PCON.png", contoursDrawn);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_PCON.png");
        }

        Mat drawnBackdropObjects = pImageROI.clone();
        MatOfPoint2f contourPoints2f;
        double perimeter;
        Point[] points;
        List<MatOfPoint> disjointPixelContourCandidates = new ArrayList<>();
        boolean foundAPixel = false;
        Rect validatedPixelRect = null;
        int validatedPixelCenterX = 0;
        for (MatOfPoint oneContour : pixelContours) {
            contourPoints2f = new MatOfPoint2f(oneContour.toArray()); // Point2f for approxPolyDP
            perimeter = Imgproc.arcLength(contourPoints2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourPoints2f, approx, 0.03 * perimeter, true); // simplify the contour
            points = approx.toArray();

            // Candidate contours for classification as a pixel must have
            // a y-coordinate in the top third of the ROI.
            Rect pixelBoundingRect = Imgproc.boundingRect(oneContour);
            if (pixelBoundingRect.y > pImageParameters.image_roi.height / 3) {
                RobotLogCommon.d(TAG, "Origin of a candidate pixel contour is not in the top third of the ROI ");
                continue;
            }

            if (points.length == 6) {
                RobotLogCommon.d(TAG, "Found a polygon with 6 sides");
                RobotLogCommon.d(TAG, "Area of a possible pixel's hexagonal contour bounding box: " + pixelBoundingRect.area());

                // Rule out out-of-size blobs that have qualified as hexagons.
                if (pixelBoundingRect.area() < pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.minBoundingBoxArea ||
                        pixelBoundingRect.area() > pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.maxBoundingBoxArea) {
                    RobotLogCommon.d(TAG, "The hexagon violates the size criteria");

                    // But collect a small blob that may be part of a disjoint pixel.
                    if (pixelBoundingRect.area() < pBackdropPixelParameters.yellowPixelBoundingBoxCriteria.maxBoundingBoxArea)
                        disjointPixelContourCandidates.add(oneContour); // include as a disjoint contour
                    continue;
                }

                // Qualify the left and right boundaries of the pixel's bounding box
                // with respect to the AprilTag's center.
                if (qualifyPixel(pImageParameters, targetAprilTagCenterX, pixelBoundingRect)) {
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
                disjointPixelContourCandidates.add(oneContour);
            }
        }

        // If we haven't found a hexagon then see if there's a
        // cluster of contours that we can assume is a pixel.
        int qualifyingDisjointCountourCenters = 0;
        int accumQualifyingCentersX = 0;
        int averageQualifyingPixelCenterX;
        int contourBoundingBoxLowestX = 1000, contourBoundingBoxHighestX = 0;
        int contourBoundingBoxLowestY = 1000, contourBoundingBoxHighestY = 0;
        if (!foundAPixel) {
            // We didn't find a complete hexagon. So look for a set of disjoint
            // contours in the top third of the ROI.
            RobotLogCommon.d(TAG, "Did not find a complete hexagon; try to find a cluster of disjoined contours that we can assume is a pixel");

            if (disjointPixelContourCandidates.size() == 0) {
                RobotLogCommon.d(TAG, "No disjoint pixel contours found");
                return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                        RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
            }

            // Within the ROI draw all of the disjoint contours.
            RobotLogCommon.d(TAG, "Number of disjoint pixel contours " + disjointPixelContourCandidates.size());
            if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
                Mat contoursDrawn = pImageROI.clone();
                ShapeDrawing.drawShapeContours(disjointPixelContourCandidates, contoursDrawn);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_DCON.png", contoursDrawn);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DCON.png");
            }

            // Qualify disjoint contours for pixel detection according to the
            // horizontal and vertical positions of their bounding boxes.
            Rect contourBoundingRect;
            int boundingBoxCenterX;
            for (MatOfPoint oneContour : disjointPixelContourCandidates) {
                contourBoundingRect = Imgproc.boundingRect(oneContour);
                boundingBoxCenterX = pImageParameters.image_roi.x + contourBoundingRect.x +
                        (contourBoundingRect.width / 2);

                // If the right boundary of the contour's bounding box is to the
                // left of the two slots that belong to the AprilTag then disqualify
                // this contour. Also disqualify this contour if the left boundary
                // of the contour's bounding box is to the right of the two slots
                // that belong to the AprilTag.
                if (!qualifyPixel(pImageParameters, targetAprilTagCenterX, contourBoundingRect)) {
                    RobotLogCommon.d(TAG, "Possible pixel contour is *not* within range of the AprilTag");
                    continue;
                }

                qualifyingDisjointCountourCenters++;
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
        if (!foundAPixel && qualifyingDisjointCountourCenters == 0) {
            RobotLogCommon.d(TAG, "Did not find a pixel (hexagon); both slots are open");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // This is the most desirable case: an AprilTag and a pixel hexagon.
        RobotLogCommon.d(TAG, "Target AprilTag center x " + targetAprilTagCenterX);
        RobotConstantsCenterStage.BackdropPixelOpenSlot backdropPixelOpenSlot;
        if (foundAPixel) {
            // Compare the relative positions of the center of the AprilTag closest
            // to the center of the full image and the center of the yellow pixel.
            RobotLogCommon.d(TAG, "Yellow pixel bounding box center x " + validatedPixelCenterX +
                    ", y " + (validatedPixelRect.y + (validatedPixelRect.height / 2)));

            backdropPixelOpenSlot = determinePixelOpenSlot(targetAprilTagCenterX, validatedPixelCenterX);
        } else { // There must be a cluster of contours that comprises a pixel.
            Rect enclosingBox = new Rect(contourBoundingBoxLowestX, contourBoundingBoxLowestY,
                    contourBoundingBoxHighestX - contourBoundingBoxLowestX,
                    contourBoundingBoxHighestY - contourBoundingBoxLowestY);
            ShapeDrawing.drawOneRectangle(enclosingBox, drawnBackdropObjects, 2);
            RobotLogCommon.d(TAG, "Draw a rectangle around the cluster of pixels " + enclosingBox);

            averageQualifyingPixelCenterX = accumQualifyingCentersX / qualifyingDisjointCountourCenters;
            RobotLogCommon.d(TAG, "Average center of disjointed pixel contours " + averageQualifyingPixelCenterX);
            backdropPixelOpenSlot = determinePixelOpenSlot(targetAprilTagCenterX, averageQualifyingPixelCenterX);
        }

        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
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
        RobotLogCommon.d(TAG, "Possible pixel contour's bounding box area: " + pPixelContourBoundingBox.area() +
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