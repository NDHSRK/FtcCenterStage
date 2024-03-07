package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.firstinspires.ftc.teamcode.xml.BackdropPixelParameters;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.List;

// Determine which of the two slots above the target AprilTag
// the robot should target to deliver a yellow pixel in Autonomous.
// Assume that the slots contain either zero or one pixels.
public class BackdropPixelRecognition {

    private static final String TAG = BackdropPixelRecognition.class.getSimpleName();

    // The next two values come from the x-coordinates of points in an image as seen
    // in Gimp. The values are percentages in relation to the x-coordinate of the
    // center of a target AprilTag.
    private static final double PIXEL_OUT_OF_RANGE_LEFT = .63;
    private static final double PIXEL_OUT_OF_RANGE_RIGHT = 1.25;

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
                                                                   AprilTagUtils.AprilTagId pTargetAprilTagId,
                                                                   double pAngleToAprilTag,
                                                                   double pCameraFieldOfView,
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
                pTargetAprilTagId, pAngleToAprilTag, pCameraFieldOfView);
    }

    private BackdropPixelReturn redChannelPathWebcam(VisionParameters.ImageParameters pImageParameters, Mat pImageROI, String pOutputFilenamePreamble,
                                                     BackdropPixelParameters pBackdropPixelParameters,
                                                     AprilTagUtils.AprilTagId pTargetAprilTagId,
                                                     double pAngleToAprilTag, double pCameraFieldOfView) {

        // Use the red channel of the image for maximum contrast - works for both the
        // yellow pixel and the white boundary around the AprilTags.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pBackdropPixelParameters.grayscaleParameters.median_target, pBackdropPixelParameters.grayscaleParameters.threshold_low);

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Go through the contours and look for a pixel in the top half of the ROI
        // and 1 -3 AprilTags in the bottom half of the ROI.
        // Store the locations of all significant objects in the full image.
        RobotLogCommon.d(TAG, "Total number of contours " + contours.size());
        RobotLogCommon.d(TAG, "Target AprilTag " + pTargetAprilTagId + ", angle to target " + pAngleToAprilTag);
        Mat drawnContours = pImageROI.clone();

        Point yellowPixelCentroid = new Point(0, 0);
        List<Point> aprilTagRectangleCentroids = new ArrayList<>();
        int roiCenterY = pImageParameters.image_roi.height / 2;
        int roiX = pImageParameters.image_roi.x;
        int roiY = pImageParameters.image_roi.y;
        boolean foundYellowPixel = false;

        MatOfPoint2f contourPoints2f;
        double perimeter;
        MatOfPoint2f approx;
        Point[] points;
        Point contourCentroid;
        double contourArea;
        int contourIndex = -1;
        for (MatOfPoint oneContour : contours) {
            contourPoints2f = new MatOfPoint2f(oneContour.toArray()); // Point2f for approxPolyDP
            perimeter = Imgproc.arcLength(contourPoints2f, true);
            approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourPoints2f, approx, 0.03 * perimeter, true); // simplify the contour
            points = approx.toArray();
            contourCentroid = ImageUtils.getContourCentroid(oneContour); // Point in the ROI
            contourArea = Imgproc.contourArea(oneContour);
            contourIndex++;

            // Look for a yellow pixel hexagon in the top half of the ROI.
            if (!foundYellowPixel && contourCentroid.y < roiCenterY) {
                RobotLogCommon.d(TAG, "Found a contour in the top half of the ROI");
                RobotLogCommon.d(TAG, "Center point in ROI " + contourCentroid + ", area " + contourArea);
                if (points.length == 6) {
                    RobotLogCommon.d(TAG, "Found a hexagon");

                    // Skip out-of-size hexagons.
                    RobotLogCommon.d(TAG, "Area of the hexagonal contour: " + contourArea);
                    if (contourArea < pBackdropPixelParameters.yellowPixelCriteria.minArea ||
                            contourArea > pBackdropPixelParameters.yellowPixelCriteria.maxArea) {
                        ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                        RobotLogCommon.d(TAG, "The hexagon violates the size criteria");
                        continue;
                    }

                    foundYellowPixel = true; // only need one pixel
                    yellowPixelCentroid = new Point(roiX + contourCentroid.x, roiY + contourCentroid.y); // full image
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 0, 255)); // red
                    RobotLogCommon.d(TAG, "Found a yellow pixel with a center point in the full image of " + yellowPixelCentroid);
                    continue;
                }

                // Sometimes approxPolyDP does not recognize a hexagon when a contour
                // contains a small indentation. No luck with adjusting the epsilon
                // factor (in spite of the example of smoothing out indentations here:
                // https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html).
                // Using Imgproc.convexHull would probably work:
                //  https://docs.opencv.org/3.4/d7/d1d/tutorial_hull.html
                // but let's just filter on area and aspect ratio.
                RobotLogCommon.d(TAG, "Area of a contour that may be hexagonal: " + contourArea);
                if (contourArea < pBackdropPixelParameters.yellowPixelCriteria.minArea ||
                        contourArea > pBackdropPixelParameters.yellowPixelCriteria.maxArea) {
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    RobotLogCommon.d(TAG, "The contour violates the size criteria for a yellow pixel");
                    continue;
                }

                // Get the bounding box to check for aspect ratio.
                Rect potentialYellowPixelBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "The potential yellow pixel's bounding box in the ROI: " + potentialYellowPixelBoundingRect);
                double aspectRatio = (double) potentialYellowPixelBoundingRect.width / potentialYellowPixelBoundingRect.height;
                if (aspectRatio < pBackdropPixelParameters.yellowPixelCriteria.minAspectRatio || aspectRatio > pBackdropPixelParameters.yellowPixelCriteria.maxAspectRatio) {
                    RobotLogCommon.d(TAG, "The potential yellow pixel's aspect ratio of " + aspectRatio + " violates the limits");
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    continue;
                }

                foundYellowPixel = true; // only need one pixel
                yellowPixelCentroid =  new Point(roiX + contourCentroid.x, roiY + contourCentroid.x); // full image
                ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 0, 255)); // red
                RobotLogCommon.d(TAG, "Found a yellow pixel with a center point in the full image of " + yellowPixelCentroid);
                continue;
            }

            // Look for AprilTag rectangles in the bottom half of the ROI.
            if (contourCentroid.y > roiCenterY) {
                RobotLogCommon.d(TAG, "Found a contour in the bottom half of the ROI");
                RobotLogCommon.d(TAG, "Center point in ROI " + contourCentroid + ", area " + contourArea);

                if (points.length == 4) {
                    RobotLogCommon.d(TAG, "Found a rectangle");
                    RobotLogCommon.d(TAG, "Area of the rectangular contour " + contourArea);

                    // Skip out-of-size rectangles.
                    if (contourArea < pBackdropPixelParameters.aprilTagRectangleCriteria.minArea ||
                            contourArea > pBackdropPixelParameters.aprilTagRectangleCriteria.maxArea) {
                        ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                        RobotLogCommon.d(TAG, "The rectangle violates the size criteria");
                        continue;
                    }

                    // Get the bounding box to check for aspect ratio.
                    Rect aprilTagBoundingRect = Imgproc.boundingRect(oneContour);
                    RobotLogCommon.d(TAG, "The AprilTag rectangle's contour bounding box in the ROI: " + aprilTagBoundingRect);

                    double aspectRatio = (double) aprilTagBoundingRect.width / aprilTagBoundingRect.height;
                    if (aspectRatio < pBackdropPixelParameters.aprilTagRectangleCriteria.minAspectRatio || aspectRatio > pBackdropPixelParameters.aprilTagRectangleCriteria.maxAspectRatio) {
                        RobotLogCommon.d(TAG, "The AprilTag rectangle's aspect ratio of " + aspectRatio + " violates the limits");
                        ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                        continue;
                    }

                    // Save the centroid of the AprilTag rectangle.
                    Point rectangleFullImageCoordinates = new Point(roiX + contourCentroid.x, roiY + contourCentroid.y);
                    aprilTagRectangleCentroids.add(rectangleFullImageCoordinates);
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 0, 255)); // red
                    RobotLogCommon.d(TAG, "Found an AprilTag rectangle on the backdrop");
                    RobotLogCommon.d(TAG, "Storing its coordinates in the full image " + rectangleFullImageCoordinates);
                    continue;
                }

                // Found some kind of a blob; draw it for information.
                RobotLogCommon.d(TAG, "The contour was not recognized as an AprilTag rectangle");
                ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
            }
        }

        // Write out all of the contours.
        if (pOutputFilenamePreamble != null && (RobotLogCommon.isLoggable("v") || writeIntermediateImageFiles)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", drawnContours);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");
        }

        // Preliminary check to see if we found a yellow pixel.
        if (!foundYellowPixel) {
            RobotLogCommon.d(TAG, "No yellow pixel found");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Preliminary check to see if we found one, two or three AprilTag rectangles.
        int numAprilTags = aprilTagRectangleCentroids.size();
        if (numAprilTags < 1 || numAprilTags > 3) {
            RobotLogCommon.d(TAG, "Need one, two or three AprilTags, found " + numAprilTags);
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Three cases: 1, 2 or 3 AprilTag rectangles.
        // 1 - Use the single AprilTag rectangle for comparison with the
        //      location of the yellow pixel.
        // Sort the AprilTag rectangles by their center point x-coordinate.
        // 3 - If the AprilTag target is 1 or 4 use the leftmost AprilTag rectangle;
        //       if the target is 2 or 5 use the center AprilTag rectangle; if the
        //       target is 3 or 6 use the rightmost AprilTag rectangle.
        // 2 - Calculate the location in pixels of the actual AprilTag using its
        //       angle and the field of view of the camera.
        //     Select the AprilTag rectangle closest to the actual April tag for
        //       comparison with the location of the yellow pixel.

        if (aprilTagRectangleCentroids.size() == 1) {
            RobotLogCommon.d(TAG, "Found 1 AprilTag rectangle");
            return findOpenSlot( (int) aprilTagRectangleCentroids.get(0).x, (int) yellowPixelCentroid.x);
        }

        // Sort the AprilTag rectangles by their x-xoordinates.
        aprilTagRectangleCentroids.sort(Comparator.comparingDouble((Point p) -> p.x));

        // Three AprilTag rectangles.
        // 3 - If the AprilTag target is 1 or 4 use the leftmost AprilTag rectangle;
        //       if the target is 2 or 5 use the center AprilTag rectangle; if the
        //       target is 3 or 6 use the rightmost AprilTag rectangle.
        if (aprilTagRectangleCentroids.size() == 3) {
            RobotLogCommon.d(TAG, "Found 3 AprilTag rectangles");
            Point aprilTagRectangleToUse;
            switch (pTargetAprilTagId) {
                case TAG_ID_1:
                case TAG_ID_4: {
                    aprilTagRectangleToUse = aprilTagRectangleCentroids.get(0);
                    break;
                }
                case TAG_ID_2:
                case TAG_ID_5: {
                    aprilTagRectangleToUse = aprilTagRectangleCentroids.get(1);
                    break;
                }
                case TAG_ID_3:
                case TAG_ID_6: {
                    aprilTagRectangleToUse = aprilTagRectangleCentroids.get(2);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid backdrop AprilTag id");
            }

            return findOpenSlot( (int) aprilTagRectangleToUse.x, (int) yellowPixelCentroid.x);
        }

        // Two AprilTag rectangles.
        // 2 - Calculate the location in pixels of the actual AprilTag using its
        //       angle and the field of view of the camera.
        //     Select the AprilTag rectangle closest to the actual April tag for
        //       comparison with the location of the yellow pixel.

        // The location of the actual AprilTag in pixels is:
        // angle to AprilTag / x pixels = (field of view / 2) / (full image width / 2)
        // Invert the sign of the angle to the AprilTag because a negative angle indicates
        // that the AprilTag is to the right of the center of the image and so should be
        // further from the left-hand edge of the image.
        // NOTE that the calculated value does not coincide exactly with the center of
        // the closest AprilTag rectangle - probably due to distortions in the image -
        // but we only need relative proximity.
        double aprilTagCenterX = (-pAngleToAprilTag * (pImageParameters.resolution_width / 2.0)) / (pCameraFieldOfView / 2.0);
        aprilTagCenterX += pImageParameters.resolution_width / 2.0;
        RobotLogCommon.d(TAG, "Calculated x-coordinate of the center of the actual AprilTag: " + aprilTagCenterX);

        // Find the AprilTagRectangle closest to the calculated center of the AprilTag.
        double closestAprilTagRectangleCenterX = findClosestAprilTagRectangle(aprilTagRectangleCentroids, aprilTagCenterX);

        RobotLogCommon.d(TAG, "X-coordinate of closest rectangle in full image " + (int) closestAprilTagRectangleCenterX);
        return findOpenSlot( (int) closestAprilTagRectangleCenterX, (int) yellowPixelCentroid.x);    }

    // Based on code from Microsoft Edge chat.
    // Find the x-coordinate of a Point a collection that is closest to
    // the constructed x-coordinate of an AprilTag.
    private double findClosestAprilTagRectangle(List<Point> pAprilTagRectangleCenters, double pAprilTagCenterX) {
        double nearest = -1;
        double bestDistanceFoundYet = Integer.MAX_VALUE;

        for (Point pAprilTagRectangleCenter : pAprilTagRectangleCenters) {
            if (pAprilTagRectangleCenter.x == pAprilTagCenterX) {
                return pAprilTagRectangleCenter.x; // If we found the desired number, return it.
            } else {
                double d = Math.abs(pAprilTagCenterX - pAprilTagRectangleCenter.x); // Calculate the difference.
                if (d < bestDistanceFoundYet) {
                    bestDistanceFoundYet = d; // Update the best distance.
                    nearest = pAprilTagRectangleCenter.x; // Update the nearest number.
                }
            }
        }
        return nearest;
    }

    // Find the slot on the backdrop above the AprilTag that is not occupied
    // by the yellow pixel.
    // We need to filter out the case where our alliance partner misplaced
    // the pixel to the left or right of the two target locations, one on
    // each side of the AprilTag. We can't use absolute pixel counts but
    // a percentage should work, e.g. if the center of the pixel is at
    // 210 x in the full image and the center of the AprilTag is at 335 x
    // then the pixel position is .63 of that of the AprilTag and is misplaced
    // to the left. If the center of the pixel is at 455 x then it is 1.36
    // more than that of the AprilTag and is misplaced to the right.
    private BackdropPixelReturn findOpenSlot(int pAprilTagRectangleCenterX, int pYellowPixelCenterX) {
        RobotLogCommon.d(TAG, "In findOpenSlot: center of AprilTag rectangle in full image " + pAprilTagRectangleCenterX);
        RobotLogCommon.d(TAG, "In findOpenSlot: center of yellow pixel in full image " + pYellowPixelCenterX);

        // If the center of the yellow pixel is to the left of the two slots
        // that belong to the AprilTag then disqualify the pixel.
        if ((pYellowPixelCenterX / (double) pAprilTagRectangleCenterX) <= PIXEL_OUT_OF_RANGE_LEFT) {
            RobotLogCommon.d(TAG, "Yellow pixel is too far to the left of the AprilTag");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // If the center of the yellow pixel is to the right of the two slots
        // that belong to the AprilTag then disqualify the pixel.
        if ((pYellowPixelCenterX / (double) pAprilTagRectangleCenterX) >= PIXEL_OUT_OF_RANGE_RIGHT) {
            RobotLogCommon.d(TAG, "Possible pixel contour is too far to the right of the AprilTag");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // The yellow pixel is in range of the AprilTag; find out whether
        // the open slot is to the left or right.
        if (pYellowPixelCenterX < pAprilTagRectangleCenterX) {
            RobotLogCommon.d(TAG, "The yellow pixel is to the left of the AprilTag");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.RIGHT);
        }

        if (pYellowPixelCenterX > pAprilTagRectangleCenterX) {
            RobotLogCommon.d(TAG, "The yellow pixel is to the right of the AprilTag");
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.LEFT);
        }


        RobotLogCommon.d(TAG, "The yellow pixel is exactly above the AprilTag");
        return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
    }

}