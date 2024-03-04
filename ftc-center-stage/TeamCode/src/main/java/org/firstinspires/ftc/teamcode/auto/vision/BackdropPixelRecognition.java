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
import org.opencv.core.Scalar;
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

    // The next two values come from the x-coordinates of points in an image as seen
    // in Gimp. The values are percentages in relation to the x-coordinate of the
    // center of a target AprilTag.
    private static final double PIXEL_OUT_OF_RANGE_LEFT = .63;
    private static final double PIXEL_OUT_OF_RANGE_RIGHT = 1.36;

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
                pTargetAprilTagId, pAngleToAprilTag);
    }

    private BackdropPixelReturn redChannelPathWebcam(VisionParameters.ImageParameters pImageParameters, Mat pImageROI, String pOutputFilenamePreamble,
                                                     BackdropPixelParameters pBackdropPixelParameters,
                                                     AprilTagUtils.AprilTagId pTargetAprilTagId,
                                                     double pAngleToAprilTag) {

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
        RobotLogCommon.d(TAG, "Total number of contours " + contours.size());
        Mat drawnContours = pImageROI.clone();

        Point yellowPixelCentroid = new Point(0, 0);
        List<Point> aprilTagCentroids = new ArrayList<>();
        int roiCenterY = pImageParameters.image_roi.height / 2;
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
            contourCentroid = ImageUtils.getContourCentroid(oneContour);
            contourArea = Imgproc.contourArea(oneContour);
            contourIndex++;

            // Look for a yellow pixel hexagon in the top half of the ROI.
            if (!foundYellowPixel && points.length == 6) {
                RobotLogCommon.d(TAG, "Found a hexagon with a center point of " + contourCentroid);

                // Skip any hexagon whose center is not in the top half of the ROI.
                if (contourCentroid.y > roiCenterY) {
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    RobotLogCommon.d(TAG, "The hexagon is not in the top half of the ROI");
                    continue;
                }

                // Skip out-of-size hexagons.
                RobotLogCommon.d(TAG, "Area of the hexagonal contour: " + contourArea);
                if (contourArea < pBackdropPixelParameters.yellowPixelAreaLimits.minArea ||
                        contourArea > pBackdropPixelParameters.yellowPixelAreaLimits.maxArea) {
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    RobotLogCommon.d(TAG, "The hexagon violates the size criteria");
                    continue;
                }

                foundYellowPixel = true; // only need one pixel
                yellowPixelCentroid = contourCentroid;
                ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 0, 255)); // red
                RobotLogCommon.d(TAG, "Found a yellow pixel on the backdrop with a center point in the ROI of " + yellowPixelCentroid);
                continue;
            }

            // Look for AprilTag rectangles in the bottom half of the ROI.
            if (points.length == 4) {
                RobotLogCommon.d(TAG, "Found a rectangle with a center point in the ROI of " + contourCentroid);

                // Skip any rectangle whose center is not in the bottom half of the ROI.
                if (contourCentroid.y < roiCenterY) {
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    RobotLogCommon.d(TAG, "The rectangle is not in the bottom half of the ROI");
                    continue;
                }

                // Skip out-of-size rectangles.
                RobotLogCommon.d(TAG, "Area of the rectangular contour " + Imgproc.contourArea(oneContour));
                if (contourArea < pBackdropPixelParameters.aprilTagRectangleAreaLimits.minArea ||
                        contourArea > pBackdropPixelParameters.aprilTagRectangleAreaLimits.maxArea) {
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    RobotLogCommon.d(TAG, "The rectangle violates the size criteria");
                    continue;
                }

                // Get the bounding box to check for aspect ratio.
                Rect aprilTagBoundingRect = Imgproc.boundingRect(oneContour);
                RobotLogCommon.d(TAG, "The possible AprilTag's contour bounding box in the ROI: " + aprilTagBoundingRect);

                double aspectRatio = (double) aprilTagBoundingRect.width / aprilTagBoundingRect.height;
                if (aspectRatio < pBackdropPixelParameters.aprilTagRectangleMinAspectRatio || aspectRatio > pBackdropPixelParameters.aprilTagRectangleMaxAspectRatio) {
                    RobotLogCommon.d(TAG, "The possible AprilTag rectangle's aspect ratio of " + aspectRatio + " violates the limits");
                    ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
                    continue;
                }

                // Save the centroid of the AprilTag rectangle.
                aprilTagCentroids.add(contourCentroid);
                ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 0, 255)); // red
                RobotLogCommon.d(TAG, "Found an AprilTag on the backdrop");
                continue;
            }

            // Found some kind of a blob; draw it for information.
            RobotLogCommon.d(TAG, "Found a contour with a center point in the ROI of " + contourCentroid + " that is neither a hexagon nor a rectangle");
            ShapeDrawing.drawOneContour(contours, contourIndex, drawnContours, new Scalar(0, 255, 0)); // green
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

        // Preliminary check to see if we found two or three AprilTag rectangles.
        int numAprilTags = aprilTagCentroids.size();
        if (numAprilTags < 2 || numAprilTags > 3) {
            RobotLogCommon.d(TAG, "Need two or three AprilTags, found " + numAprilTags);
            return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                    RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
        }

        // Two cases: 2 or 3 AprilTag rectangles.
        // Sort the AprilTag rectangles by their center point x-coordinate.
        // 3 - If the AprilTag target is 1 or 4 use the leftmost AprilTag rectangle;
        //       if the target is 2 or 5 use the center AprilTag rectangle; if the
        //       target is 3 or 6 use the rightmost AprilTag rectangle.
        // 2 - How many AprilTag rectangles are on the same side of image center as the
        //     target AprilTag?
        //   1 - found the correct AprilTag
        //   2 - if the target AprilTag is on the right-hand side of the backdrop (3, 6)
        //         use the AprilTag rectangle furthest to the right
        //       if the target AprilTag is on the left-hand side of the backdrop (1, 4)
        //         use the AprilTag rectangle furthest to the left
        //       if the target AprilTags is in the center (2, 5) use the AprilTag
        //         rectangle closest to the center

        // Sort the AprilTag rectangles by their x-xoordinates.
        aprilTagCentroids.sort(Comparator.comparingDouble((Point p) -> p.x));
        int aprilTagAngleSign = (int) Math.signum(pAngleToAprilTag);
        int imageCenterX = pImageParameters.resolution_width / 2;
        int roiX = pImageParameters.image_roi.x;

        // Three AprilTag rectangles.
        // 3 - If the AprilTag target is 1 or 4 use the leftmost AprilTag rectangle;
        //       if the target is 2 or 5 use the center AprilTag rectangle; if the
        //       target is 3 or 6 use the rightmost AprilTag rectangle.
        if (aprilTagCentroids.size() == 3) {
            RobotLogCommon.d(TAG, "Found 3 AprilTag rectangles");
            Point aprilTagRectangleToUse;
            switch (pTargetAprilTagId) {
                case TAG_ID_1:
                case TAG_ID_4: {
                    aprilTagRectangleToUse = aprilTagCentroids.get(0);
                    break;
                }
                case TAG_ID_2:
                case TAG_ID_5: {
                    aprilTagRectangleToUse = aprilTagCentroids.get(1);
                    break;
                }
                case TAG_ID_3:
                case TAG_ID_6: {
                    aprilTagRectangleToUse = aprilTagCentroids.get(2);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid backdrop AprilTag id");
            }

            return findOpenSlot(roiX + (int) aprilTagRectangleToUse.x, roiX + (int) yellowPixelCentroid.x);
        }

        // Two AprilTagRectangles.
        // 2 - How many AprilTag rectangles are on the same side of image center as the
        //     target AprilTag?
        //   1 - found the correct AprilTag
        if (aprilTagCentroids.size() == 2) {
            RobotLogCommon.d(TAG, "Found 2 AprilTag rectangles");
            RobotLogCommon.d(TAG, "Apply the primary location filter");
            List<Point> filteredAprilTagCentroids = new ArrayList<>();
            for (Point oneAprilTagCentroid : aprilTagCentroids) {
                Point updatedCentroid = new Point(roiX + oneAprilTagCentroid.x, oneAprilTagCentroid.y);
                if (aprilTagAngleSign < 0 && updatedCentroid.x > imageCenterX) {
                    RobotLogCommon.d(TAG, "AprilTag rectangle passed the primary location filter; center point in full image " + updatedCentroid);
                    filteredAprilTagCentroids.add(updatedCentroid);
                } else if (aprilTagAngleSign >= 0 && updatedCentroid.x <= imageCenterX) {
                    filteredAprilTagCentroids.add(updatedCentroid);
                    RobotLogCommon.d(TAG, "AprilTag rectangle passed the primary location filter; center point in full image" + updatedCentroid);
                } else
                    RobotLogCommon.d(TAG, "AprilTag rectangle did not pass the primary location filter; center point in full image " + updatedCentroid);
            }

            if (filteredAprilTagCentroids.isEmpty()) {
                RobotLogCommon.d(TAG, "No AprilTag rectangles passed the primary location filter");
                return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL,
                        RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
            }

            if (filteredAprilTagCentroids.size() == 1) {
                RobotLogCommon.d(TAG, "A single AprilTag rectangle passed the primary location filter");
                return findOpenSlot((int) filteredAprilTagCentroids.get(0).x, roiX + (int) yellowPixelCentroid.x);
            }

            // There are two AprilTag rectangles whose centers are in the same half of the
            // image as the target AprilTag.
            //   2 - if the target AprilTag is on the right-hand side of the backdrop (3, 6)
            //         use the AprilTag rectangle furthest to the right
            //       if the target AprilTag is on the left-hand side of the backdrop (1, 4)
            //         use the AprilTag rectangle furthest to the left
            //       if the target AprilTags is in the center (2, 5) use the AprilTag
            //         rectangle closest to the center
            RobotLogCommon.d(TAG, "Apply the secondary location filter");
            Point aprilTagRectangleToUse;
            switch (pTargetAprilTagId) {
                case TAG_ID_3:
                case TAG_ID_6: {
                    aprilTagRectangleToUse = filteredAprilTagCentroids.get(1);
                    break;
                }
                case TAG_ID_1:
                case TAG_ID_4: {
                    aprilTagRectangleToUse = filteredAprilTagCentroids.get(0);
                    break;
                }
                case TAG_ID_2:
                case TAG_ID_5: {
                    if (Math.abs(imageCenterX - filteredAprilTagCentroids.get(0).x) <=
                            Math.abs(imageCenterX - filteredAprilTagCentroids.get(1).x))
                        aprilTagRectangleToUse = filteredAprilTagCentroids.get(0);
                    else
                        aprilTagRectangleToUse = filteredAprilTagCentroids.get(1);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid backdrop AprilTag id");
            }

            RobotLogCommon.d(TAG, "A single AprilTag rectangle passed the secondary location filter");
            RobotLogCommon.d(TAG, "X-coordinate of rectangle in full image " + (int) aprilTagRectangleToUse.x);
            return findOpenSlot((int) aprilTagRectangleToUse.x, roiX + (int) yellowPixelCentroid.x);
        }

        RobotLogCommon.d(TAG, "No AprilTag rectangle passed the secondary location filter");
        return new BackdropPixelReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, RobotConstantsCenterStage.BackdropPixelOpenSlot.ANY_OPEN_SLOT);
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