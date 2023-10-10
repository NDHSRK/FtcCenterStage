package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public class TeamPropRecognition {

    private static final String TAG = TeamPropRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;
    private EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows;

    public TeamPropRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
        alliance = pAlliance;
    }

    // Returns the result of image analysis.
    public TeamPropReturn recognizeTeamProp(ImageProvider pImageProvider,
                                            VisionParameters.ImageParameters pImageParameters,
                                            TeamPropParameters pTeamPropParameters,
                                            RobotConstantsCenterStage.TeamPropRecognitionPath pTeamPropRecognitionPath) throws InterruptedException {
        RobotLogCommon.d(TAG, "In TeamPropRecognition.recognizeTeamProp");

        spikeWindows = pTeamPropParameters.getSpikeWindows();

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> teamPropImage = pImageProvider.getImage();
        if (teamPropImage == null)
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getDateTimeStamp(teamPropImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(teamPropImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pTeamPropRecognitionPath);
        switch (pTeamPropRecognitionPath) {
            case COLOR_CHANNEL_CIRCLES: {
                return colorChannelCirclesPath(imageROI, outputFilenamePreamble, pTeamPropParameters.colorChannelCirclesParameters);
            }
            case COLOR_CHANNEL_FEATURES: {
                return colorChannelFeaturesPath(imageROI, outputFilenamePreamble, pTeamPropParameters.colorChannelFeaturesParameters);
            }
            case COLOR_CHANNEL_CONTOURS: {
                return colorChannelContoursPath(imageROI, outputFilenamePreamble, pTeamPropParameters.colorChannelContoursParameters);
            }
            case COLOR_CHANNEL_BRIGHT_SPOT: {
                return colorChannelBrightSpotPath(imageROI, outputFilenamePreamble, pTeamPropParameters.brightSpotParameters);
            }
            case GRAYSCALE_BRIGHT_SPOT: {
                return grayscaleBrightSpotPath(imageROI, outputFilenamePreamble, pTeamPropParameters.brightSpotParameters);
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private TeamPropReturn colorChannelCirclesPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                   TeamPropParameters.ColorChannelCirclesParameters pColorChannelCirclesParameters) {
        Mat split = splitChannels(pImageROI, pColorChannelCirclesParameters.grayParameters, pOutputFilenamePreamble);

        // Apply a 2d filter to sharpen the image.
        Mat sharp = sharpen(split, pOutputFilenamePreamble);

        // Remove noise by Gaussian blurring.
        Imgproc.GaussianBlur(sharp, sharp, new Size(5, 5), 0);

        // Support both full and partial circles depending upon the ROI
        // and the parameters in the XML file.
        // See https://stackoverflow.com/questions/20698613/detect-semicircle-in-opencv
        // dp = 1, minDist = 60, param1 = 200, param2 = 20, 0, 0);
        // Perform HoughCircles recognition
        Mat circles = new Mat();
        Imgproc.HoughCircles(sharp, circles, Imgproc.HOUGH_GRADIENT,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.dp,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.minDist,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.param1,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.param2,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.minRadius,
                pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.maxRadius);

        RobotLogCommon.d(TAG, "Number of circles " + circles.cols());

        // If no circles were found then assume that the prop is outside
        // of the ROI; use the NPOS position.
        Mat propOut = pImageROI.clone();
        if (circles.cols() == 0) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "No circles found; Team Prop location assumed as " + nposWindow.second);
            drawSpikeWindows(propOut, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        int numberOfTeamPropsFound = 0;
        int largestRadius = -1;
        Point centerOfLargestCircle = null;
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            int radius = (int) Math.round(c[2]);

            RobotLogCommon.d(TAG, "Found a circle with center at x " + center.x + ", y " + center.y + ", radius " + radius);

            // Always draw a circle outline around the contour.
            Imgproc.circle(propOut, center, radius, new Scalar(255, 0, 255), 3, 8, 0);

            // Apply the filters.
            // Test for minimum radius, maximum radius.
            if (radius < pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.minRadius) {
                // Circle is too small.
                RobotLogCommon.d(TAG, "False positive: circle too small, radius: " + radius);
                continue;
            }

            if (radius > pColorChannelCirclesParameters.houghCirclesFunctionCallParameters.maxRadius) {
                // Circle is too large.
                RobotLogCommon.d(TAG, "False positive: circle too large, radius: " + radius);
                continue;
            }

            // Passed all filters. Found a circle that might be a team prop.
            numberOfTeamPropsFound++;
            RobotLogCommon.d(TAG, "Found a candidate for a team prop, radius " + radius);
            if (radius > largestRadius) {
                largestRadius = radius;
                centerOfLargestCircle = center;
            }
        }

        // We found at least one circle - but make sure that we've
        // also passed all of the filters.
        if (numberOfTeamPropsFound == 0) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "No circles passed the filters; Team Prop location assumed as " + nposWindow.second);
            drawSpikeWindows(propOut, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        // Draw a black circle at the center of the largest circle.
        Imgproc.circle(propOut, Objects.requireNonNull(centerOfLargestCircle), 10, new Scalar(0, 0, 0), 4);

        String teamPropFilename = pOutputFilenamePreamble + "_CIR.png";
        RobotLogCommon.d(TAG, "Writing " + teamPropFilename);
        Imgcodecs.imwrite(teamPropFilename, propOut);
        RobotLogCommon.d(TAG, "Number of candidate team props found: " + numberOfTeamPropsFound);

        if (numberOfTeamPropsFound > pColorChannelCirclesParameters.maxCircles) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "Number of circles (" + numberOfTeamPropsFound + ") " +
                    "exceeds the maximum of " + pColorChannelCirclesParameters.maxCircles);
            drawSpikeWindows(propOut, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, nposWindow.second);
        }

        return lookThroughWindows(propOut, centerOfLargestCircle, pOutputFilenamePreamble);
    }

    private TeamPropReturn colorChannelFeaturesPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                    TeamPropParameters.ColorChannelFeaturesParameters pFeaturesParameters) {
        Mat split = splitChannels(pImageROI, pFeaturesParameters.grayParameters, pOutputFilenamePreamble);

        // Apply a 2d filter to sharpen the image.
        Mat sharp = sharpen(split, pOutputFilenamePreamble);

        MatOfPoint corners = new MatOfPoint();
        Imgproc.goodFeaturesToTrack(sharp, corners, pFeaturesParameters.maxCorners, pFeaturesParameters.qualityLevel, 0, new Mat(), 2, false, 0.04);
        Mat featuresOut = pImageROI.clone();
        for (int i = 0; i < corners.height(); i++) {
            Imgproc.circle(featuresOut, new Point(corners.get(i, 0)), 3, new Scalar(0, 255, 0));
        }

        // For convenience convert the MatOfPoint into a List,
        // sort it separately by the x-coordinate of the Points
        // and then the y-coordinates of the Points, and make
        // a composite median Point.
        List<Point> cornersListX = corners.toList();
        cornersListX.sort(Comparator.comparing(point -> point.x));
        List<Point> cornersListY = new ArrayList<>(cornersListX);
        cornersListY.sort(Comparator.comparing(point -> point.y));

        int size = cornersListX.size();
        if (size == 0) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "No features found");
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, nposWindow.second);
        }

        double compositeMedianFeatureX;
        if (size % 2 == 0)
            compositeMedianFeatureX = (cornersListX.get(size / 2).x + cornersListX.get(size / 2 - 1).x) / 2;
        else
            compositeMedianFeatureX = cornersListX.get(size / 2).x;

        double compositeMedianFeatureY;
        if (size % 2 == 0)
            compositeMedianFeatureY = (cornersListY.get(size / 2).y + cornersListY.get(size / 2 - 1).y) / 2;
        else
            compositeMedianFeatureY = cornersListY.get(size / 2).y;

        Point compositePoint = new Point(compositeMedianFeatureX, compositeMedianFeatureY);
        RobotLogCommon.d(TAG, "Composite features median x " + compositeMedianFeatureX + " y " +
                compositeMedianFeatureY);
        Imgproc.circle(featuresOut, compositePoint, 7, new Scalar(0, 255, 255), 4);

        String featuresFilename = pOutputFilenamePreamble + "_FEATURES.png";
        RobotLogCommon.d(TAG, "Writing " + featuresFilename);
        Imgcodecs.imwrite(featuresFilename, featuresOut);

        return lookThroughWindows(featuresOut, compositePoint, pOutputFilenamePreamble);
    }

    // The inverted red channel of a blue ball has very
    // good contrast with the background. Red (inverted blue) also has good
    // contrast with the background (red stripe 194, ball 254 in Gimp).
    private TeamPropReturn colorChannelContoursPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                    TeamPropParameters.ColorChannelContoursParameters pColorChannelContoursParameters) {
        Mat split = splitChannels(pImageROI, pColorChannelContoursParameters.grayParameters, pOutputFilenamePreamble);

        // Apply a 2d filter to sharpen the image.
        Mat sharp = sharpen(split, pOutputFilenamePreamble);
        Mat thresholded = ImageUtils.applyGrayThreshold(sharp, pColorChannelContoursParameters.grayParameters.threshold_low);

        Optional<Pair<Integer, MatOfPoint>> targetContour = ImageUtils.getLargestContour(pImageROI, thresholded, pOutputFilenamePreamble);
        if (!targetContour.isPresent()) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "No contours found");
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, nposWindow.second);
        }

        MatOfPoint largestContour = targetContour.get().second;
        double contourArea = Imgproc.contourArea(largestContour);
        RobotLogCommon.d(TAG, "Area of largest contour: " + contourArea);

        // Within the ROI draw a circle around the largest contour.
        Mat enclosingCircleOut = pImageROI.clone();
        float[] radius = new float[1];
        Point center = new Point();
        Imgproc.minEnclosingCircle(new MatOfPoint2f(largestContour.toArray()), center, radius);
        Imgproc.circle(enclosingCircleOut, center, (int) radius[0], new Scalar(0, 0, 0), 4);

        // Check the area of the enclosing circle around the largest contour.
        double enclosingCircleArea = Math.PI * Math.pow(radius[0], 2);
        RobotLogCommon.d(TAG, "Area of enclosing circle: " + enclosingCircleArea);

        if (contourArea < pColorChannelContoursParameters.minArea ||
                contourArea > pColorChannelContoursParameters.maxArea) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "The largest contour violates the size criteria");
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, nposWindow.second);
        }

        // Get the center point of the largest contour.
        Point contourCentroid = ImageUtils.getContourCentroid(largestContour);
        return lookThroughWindows(enclosingCircleOut, contourCentroid, pOutputFilenamePreamble);
    }

    private TeamPropReturn colorChannelBrightSpotPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                      TeamPropParameters.BrightSpotParameters pBrightSpotParameters) {
        Mat split = splitChannels(pImageROI, pBrightSpotParameters.grayParameters, pOutputFilenamePreamble);

        // Apply a 2d filter to sharpen the image.
        Mat sharp = sharpen(split, pOutputFilenamePreamble);

        // See --
        // https://pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
        Mat bright = new Mat();
        Imgproc.GaussianBlur(sharp, bright, new Size(pBrightSpotParameters.blurKernel, pBrightSpotParameters.blurKernel), 0);

        String blurFilename = pOutputFilenamePreamble + "_BLUR.png";
        RobotLogCommon.d(TAG, "Writing " + blurFilename);
        Imgcodecs.imwrite(blurFilename, bright);

        Core.MinMaxLocResult brightResult = Core.minMaxLoc(bright);
        Mat brightSpotOut = pImageROI.clone();
        Imgproc.circle(brightSpotOut, brightResult.maxLoc, (int) pBrightSpotParameters.blurKernel, new Scalar(0, 255, 0));

        String brightSpotFilename = pOutputFilenamePreamble + "_BRIGHT.png";
        RobotLogCommon.d(TAG, "Writing " + brightSpotFilename);
        Imgcodecs.imwrite(brightSpotFilename, brightSpotOut);

        return lookThroughWindows(brightSpotOut, brightResult.maxLoc, pOutputFilenamePreamble);
    }

    private TeamPropReturn grayscaleBrightSpotPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                   TeamPropParameters.BrightSpotParameters pBrightSpotParameters) {
        Mat gray = new Mat();
        Imgproc.cvtColor(pImageROI, gray, Imgproc.COLOR_BGR2GRAY);

        String grayFilename = pOutputFilenamePreamble + "_GRAY.png";
        RobotLogCommon.d(TAG, "Writing " + grayFilename);
        Imgcodecs.imwrite(grayFilename, gray);

        Core.bitwise_not(gray, gray); // invert for better contrast

        String grayInvertedFilename = pOutputFilenamePreamble + "_GRAY_INVERTED.png";
        RobotLogCommon.d(TAG, "Writing " + grayInvertedFilename);
        Imgcodecs.imwrite(grayInvertedFilename, gray);

        Mat graySharp = sharpen(gray, pOutputFilenamePreamble + "_GRAY");

        Mat brightGray = new Mat();
        Imgproc.GaussianBlur(graySharp, brightGray, new Size(pBrightSpotParameters.blurKernel, pBrightSpotParameters.blurKernel), 0);

        String blurFilename = pOutputFilenamePreamble + "_GRAY__BLUR.png";
        RobotLogCommon.d(TAG, "Writing " + blurFilename);
        Imgcodecs.imwrite(blurFilename, brightGray);

        Core.MinMaxLocResult brightGrayResult = Core.minMaxLoc(brightGray);
        Mat brightGraySpotGrayOut = pImageROI.clone();
        Imgproc.circle(brightGraySpotGrayOut, brightGrayResult.maxLoc, (int) pBrightSpotParameters.blurKernel, new Scalar(0, 255, 0));

        String brightSpotGrayFilename = pOutputFilenamePreamble + "_GRAY_BRIGHT.png";
        RobotLogCommon.d(TAG, "Writing " + brightSpotGrayFilename);
        Imgcodecs.imwrite(brightSpotGrayFilename, brightGraySpotGrayOut);

        return lookThroughWindows(brightGraySpotGrayOut, brightGrayResult.maxLoc, pOutputFilenamePreamble);
    }

    // Look through the left and right windows and determine if the team prop
    // is in the left window, the right window, or neither. Also draw the boundaries
    // of the windows.
    private TeamPropReturn lookThroughWindows(Mat pPropOut, Point pCenterOfObject,
                                              String pOutputFilenamePreamble) {
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
        RobotConstantsCenterStage.TeamPropLocation foundLocation;

        if (leftWindow == null || rightWindow == null || nposWindow == null)
            throw new AutonomousRobotException(TAG, "Failed sanity check: no data for at least one of the spike windows");

        // Try the left window.
        if (pCenterOfObject.x >= leftWindow.first.x && pCenterOfObject.x < leftWindow.first.x + leftWindow.first.width) {
            RobotLogCommon.d(TAG, "Success: Team Prop found in the left spike window: location " + leftWindow.second);
            foundLocation = leftWindow.second;
        } else
            // Try the right window.
            if (pCenterOfObject.x >= rightWindow.first.x && pCenterOfObject.x < rightWindow.first.x + rightWindow.first.width) {
                RobotLogCommon.d(TAG, "Success: Team Prop found in the right spike window: location " + rightWindow.second);
                foundLocation = rightWindow.second;
            } else {
                RobotLogCommon.d(TAG, "Team Prop not found in the left or right window: assuming location " + nposWindow.second);
                foundLocation = nposWindow.second;
            }

        // Draw the spike windows on the ROI with the circles.
        drawSpikeWindows(pPropOut, pOutputFilenamePreamble);

        return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, foundLocation);
    }

    // Split the original image ROI into its BGR channels. The current alliance
    // determines which channel to pre-process and return.
    private Mat splitChannels(Mat pImageROI, VisionParameters.GrayParameters pGrayParameters, String pOutputFilenamePreamble) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2
        Mat selectedChannel;
        switch (alliance) {
            case RED: {
                // The inversion of the blue channel gives better contrast
                // than the red channel.
                selectedChannel = channels.get(0);
                Core.bitwise_not(selectedChannel, selectedChannel);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_BLUE_INVERTED.png", selectedChannel);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BLUE_INVERTED.png");
                break;
            }
            case BLUE: {
                // The inversion of the red channel gives better contrast
                // than the blue channel.
                selectedChannel = channels.get(2);
                Core.bitwise_not(selectedChannel, selectedChannel);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_INVERTED.png", selectedChannel);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_INVERTED.png");
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Alliance must be RED or BLUE");
        }

        // Always adjust the grayscale.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(selectedChannel,
                pGrayParameters.median_target);

        Imgproc.erode(adjustedGray, adjustedGray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(adjustedGray, adjustedGray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        return adjustedGray;
    }

    // ## This sharpening filter makes a difference in marginal cases.
    // From OpencvTestbed3 (cpp) GrayscaleTechnique
    // From https://stackoverflow.com/questions/27393401/opencv-in-java-for-image-filtering
    private Mat sharpen(Mat pDullMat, String pOutputFilenamePreamble) {
        int kernelSize = 3;
        Mat kernel = new Mat(kernelSize, kernelSize, CvType.CV_32F) {
            {
                put(0, 0, 0);
                put(0, 1, -1);
                put(0, 2, 0);

                put(1, 0, -1);
                put(1, 1, 5);
                put(1, 2, -1);

                put(2, 0, 0);
                put(2, 1, -1);
                put(2, 2, 0);
            }
        };

        Mat sharpMat = new Mat();
        Imgproc.filter2D(pDullMat, sharpMat, -1, kernel);

        String sharpFilename = pOutputFilenamePreamble + "_SHARP.png";
        RobotLogCommon.d(TAG, "Writing " + sharpFilename);
        Imgcodecs.imwrite(sharpFilename, sharpMat);

        return sharpMat;
    }

    private void drawSpikeWindows(Mat pPropOut, String pOutputFilenamePreamble) {
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);

        // Draw the spike windows on the ROI with the circles
        // so that we can see their placement during debugging.
        // params Mat, Point upperLeft, Point lowerRight, Scalar color, int thickness

        Point leftWindowUpperLeft = new Point(leftWindow.first.x, leftWindow.first.y);
        Point leftWindowLowerRight = new Point(leftWindow.first.x + leftWindow.first.width,
                leftWindow.first.y + leftWindow.first.height);

        Point rightWindowUpperLeft = new Point(rightWindow.first.x, rightWindow.first.y);
        Point rightWindowLowerRight = new Point(rightWindow.first.x + rightWindow.first.width,
                rightWindow.first.y + rightWindow.first.height);

        Imgproc.rectangle(pPropOut, leftWindowUpperLeft, leftWindowLowerRight, new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(pPropOut, rightWindowUpperLeft, rightWindowLowerRight, new Scalar(0, 255, 0), 3);

        String teamPropFilename = pOutputFilenamePreamble + "_PROP.png";
        RobotLogCommon.d(TAG, "Writing " + teamPropFilename);
        Imgcodecs.imwrite(teamPropFilename, pPropOut);
    }
}