package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.Objects;

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
                                            RobotConstantsCenterStage.TeamPropRecognitionPath pTeamPropRecognitionPath,
                                            TeamPropParameters pTeamPropParameters,
                                            SpikeWindowMapping pSpikeWindowMapping) throws InterruptedException {
        RobotLogCommon.d(TAG, "In TeamPropRecognition.recognizeTeamProp");

        spikeWindows = pSpikeWindowMapping.spikeWindows;

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> teamPropImage = pImageProvider.getImage();
        if (teamPropImage == null)
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getDateTimeStamp(teamPropImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pSpikeWindowMapping.imageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(teamPropImage.first, outputFilenamePreamble, pSpikeWindowMapping.imageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pTeamPropRecognitionPath);
        switch (pTeamPropRecognitionPath) {
            case COLOR_CHANNEL_CIRCLES: {
                return colorChannelCirclesPath(imageROI, outputFilenamePreamble, pTeamPropParameters.colorChannelCirclesParameters);
            }

            //## Not supported in Android.
            //case COLOR_CHANNEL_FEATURES: {
            //case COLOR_CHANNEL_CONTOURS: {

            case COLOR_CHANNEL_PIXEL_COUNT: {
                return colorChannelPixelCountPath(imageROI, outputFilenamePreamble, pTeamPropParameters.colorChannelPixelCountParameters);
            }

            //## Share code with COLOR_CHANNEL_PIXEL_COUNT but start with
            // a grayscale image, e.g. one from an ArduCam OV9281.
            // case GRAYSCALE_PIXEL_COUNT

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
        Mat split = splitAndInvertChannels(pImageROI, alliance, pColorChannelCirclesParameters.grayParameters, pOutputFilenamePreamble);

        // Sharpening the image does not improve the results.
        // Apply a 2d filter to sharpen the image.
        //Mat sharp = sharpen(split, pOutputFilenamePreamble);

        // Remove noise by Gaussian blurring.
        Imgproc.GaussianBlur(split, split, new Size(5, 5), 0);

        // Support both full and partial circles depending upon the ROI
        // and the parameters in the XML file.
        // See https://stackoverflow.com/questions/20698613/detect-semicircle-in-opencv
        // dp = 1, minDist = 60, param1 = 200, param2 = 20, 0, 0);
        // Perform HoughCircles recognition
        Mat circles = new Mat();
        Imgproc.HoughCircles(split, circles, Imgproc.HOUGH_GRADIENT,
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
            SpikeWindowUtils.drawSpikeWindows(propOut, spikeWindows, pOutputFilenamePreamble);
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
            SpikeWindowUtils.drawSpikeWindows(propOut, spikeWindows, pOutputFilenamePreamble);
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
            SpikeWindowUtils.drawSpikeWindows(propOut, spikeWindows, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, nposWindow.second);
        }

        return lookThroughWindows(propOut, centerOfLargestCircle, pOutputFilenamePreamble);
    }

    private TeamPropReturn colorChannelPixelCountPath(Mat pImageROI, String pOutputFilenamePreamble, TeamPropParameters.ColorChannelPixelCountParameters pColorChannelPixelCountParameters) {
        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        int allianceMinWhitePixelCount;
        switch (alliance) {
            case RED: {
                allianceGrayParameters = pColorChannelPixelCountParameters.redGrayParameters;
                allianceMinWhitePixelCount = pColorChannelPixelCountParameters.redMinWhitePixelCount;
                break;
            }
            case BLUE: {
                allianceGrayParameters = pColorChannelPixelCountParameters.blueGrayParameters;
                allianceMinWhitePixelCount = pColorChannelPixelCountParameters.blueMinWhitePixelCount;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        // Threshold the image: set pixels over the threshold value to white.
        Mat split = splitAndInvertChannels(pImageROI, alliance, allianceGrayParameters, pOutputFilenamePreamble);
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(split, thresholded,
                Math.abs(allianceGrayParameters.threshold_low),    // threshold value
                255,   // white
                allianceGrayParameters.threshold_low >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type
        RobotLogCommon.d(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        // Get the white pixel count for both the left and right
        // spike windows.
        Rect leftSpikeWindowBoundary = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT).first;
        Mat leftSpikeWindow = thresholded.submat(leftSpikeWindowBoundary);
        int leftNonZeroCount = Core.countNonZero(leftSpikeWindow);
        RobotLogCommon.d(TAG, "Left spike window white pixel count " + leftNonZeroCount);

        Rect rightSpikeWindowBoundary = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT).first;
        Mat rightSpikeWindow = thresholded.submat(rightSpikeWindowBoundary);
        int rightNonZeroCount = Core.countNonZero(rightSpikeWindow);
        RobotLogCommon.d(TAG, "Right spike window white pixel count " + rightNonZeroCount);

        // If both counts are less than the minimum then we infer that
        // the Team Prop is in the third (non-visible) spike window.
        if (leftNonZeroCount < allianceMinWhitePixelCount &&
                rightNonZeroCount < allianceMinWhitePixelCount) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "White pixel counts for the left and right spike windows were under the threshold");
            SpikeWindowUtils.drawSpikeWindows(pImageROI.clone(), spikeWindows, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        // Compare the white pixel count in the left and right spike
        // windows against each other.
        Mat pixelCountOut = pImageROI.clone();
        if (leftNonZeroCount >= rightNonZeroCount) {
            Point leftSpikeWindowCentroid = new Point((leftSpikeWindowBoundary.x + leftSpikeWindowBoundary.width) / 2.0,
                    (leftSpikeWindowBoundary.y + leftSpikeWindowBoundary.height) / 2.0);
            RobotLogCommon.d(TAG, "Center of left spike window " + leftSpikeWindowCentroid);

            Imgproc.circle(pixelCountOut, leftSpikeWindowCentroid, 10, new Scalar(0, 255, 0));
            return lookThroughWindows(pixelCountOut, leftSpikeWindowCentroid, pOutputFilenamePreamble);
        }

        // Go with the right spike window.
        Point rightSpikeWindowCentroid = new Point(rightSpikeWindowBoundary.x + (rightSpikeWindowBoundary.width / 2.0),
                (rightSpikeWindowBoundary.y + rightSpikeWindowBoundary.height) / 2.0);
        RobotLogCommon.d(TAG, "Center of right spike window " + rightSpikeWindowCentroid);

        Imgproc.circle(pixelCountOut, rightSpikeWindowCentroid, 10, new Scalar(0, 255, 0));
        return lookThroughWindows(pixelCountOut, rightSpikeWindowCentroid, pOutputFilenamePreamble);
    }

    private TeamPropReturn colorChannelBrightSpotPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                      TeamPropParameters.BrightSpotParameters pBrightSpotParameters) {
        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        double allianceBlurKernel;
        switch (alliance) {
            case RED: {
                allianceGrayParameters = pBrightSpotParameters.redGrayParameters;
                allianceBlurKernel = pBrightSpotParameters.redBlurKernel;
                break;
            }
            case BLUE: {
                allianceGrayParameters = pBrightSpotParameters.blueGrayParameters;
                allianceBlurKernel = pBrightSpotParameters.blueBlurKernel;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "colorChannelBrightSpotPath requires an alliance selection");
        }

        // Threshold the image: set pixels over the threshold value to white.
        Mat split = splitAndInvertChannels(pImageROI, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Sharpening the image does not improve the results.
        //Mat sharp = sharpen(split, pOutputFilenamePreamble);

        // See --
        // https://pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
        Mat bright = new Mat();
        Imgproc.GaussianBlur(split, bright, new Size(allianceBlurKernel, allianceBlurKernel), 0);

        String blurFilename = pOutputFilenamePreamble + "_BLUR.png";
        RobotLogCommon.d(TAG, "Writing " + blurFilename);
        Imgcodecs.imwrite(blurFilename, bright);

        Core.MinMaxLocResult brightResult = Core.minMaxLoc(bright);
        RobotLogCommon.d(TAG, "Bright spot location " + brightResult.maxLoc + ", value " + brightResult.maxVal);

        Mat brightSpotOut = pImageROI.clone();
        Imgproc.circle(brightSpotOut, brightResult.maxLoc, (int) allianceBlurKernel, new Scalar(0, 255, 0));

        String brightSpotFilename = pOutputFilenamePreamble + "_BRIGHT.png";
        RobotLogCommon.d(TAG, "Writing " + brightSpotFilename);
        Imgcodecs.imwrite(brightSpotFilename, brightSpotOut);

        // If the bright spot is under the threshold then assume no Team Prop is present.
        if (brightResult.maxVal < allianceGrayParameters.threshold_low) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "Bright spot value was under the threshold");
            SpikeWindowUtils.drawSpikeWindows(pImageROI.clone(), spikeWindows, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        return lookThroughWindows(brightSpotOut, brightResult.maxLoc, pOutputFilenamePreamble);
    }

    private TeamPropReturn grayscaleBrightSpotPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                   TeamPropParameters.BrightSpotParameters pBrightSpotParameters) {
        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        double allianceBlurKernel;
        switch (alliance) {
            case RED: {
                allianceGrayParameters = pBrightSpotParameters.redGrayParameters;
                allianceBlurKernel = pBrightSpotParameters.redBlurKernel;
                break;
            }
            case BLUE: {
                allianceGrayParameters = pBrightSpotParameters.blueGrayParameters;
                allianceBlurKernel = pBrightSpotParameters.blueBlurKernel;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "colorChannelBrightSpotPath requires an alliance selection");
        }

        Mat gray = new Mat();
        Imgproc.cvtColor(pImageROI, gray, Imgproc.COLOR_BGR2GRAY);

        String grayFilename = pOutputFilenamePreamble + "_GRAY.png";
        RobotLogCommon.d(TAG, "Writing " + grayFilename);
        Imgcodecs.imwrite(grayFilename, gray);

        Core.bitwise_not(gray, gray); // invert for better contrast

        String grayInvertedFilename = pOutputFilenamePreamble + "_GRAY_INVERTED.png";
        RobotLogCommon.d(TAG, "Writing " + grayInvertedFilename);
        Imgcodecs.imwrite(grayInvertedFilename, gray);

        // Sharpening the image does not improve the results.
        //Mat graySharp = sharpen(gray, pOutputFilenamePreamble + "_GRAY");

        Mat brightGray = new Mat();
        Imgproc.GaussianBlur(gray, brightGray, new Size(allianceBlurKernel, allianceBlurKernel), 0);

        String blurFilename = pOutputFilenamePreamble + "_GRAY_BLUR.png";
        RobotLogCommon.d(TAG, "Writing " + blurFilename);
        Imgcodecs.imwrite(blurFilename, brightGray);

        Core.MinMaxLocResult brightGrayResult = Core.minMaxLoc(brightGray);
        RobotLogCommon.d(TAG, "Bright spot location " + brightGrayResult.maxLoc + ", value " + brightGrayResult.maxVal);

        Mat brightGraySpotOut = pImageROI.clone();
        Imgproc.circle(brightGraySpotOut, brightGrayResult.maxLoc, (int) allianceBlurKernel, new Scalar(0, 255, 0));

        String brightSpotGrayFilename = pOutputFilenamePreamble + "_GRAY_BRIGHT.png";
        RobotLogCommon.d(TAG, "Writing " + brightSpotGrayFilename);
        Imgcodecs.imwrite(brightSpotGrayFilename, brightGraySpotOut);

        // If the bright spot is under the threshold then assume no Team Prop is present.
        if (brightGrayResult.maxVal < allianceGrayParameters.threshold_low) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "Bright spot value was under the threshold");
            SpikeWindowUtils.drawSpikeWindows(pImageROI.clone(), spikeWindows, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        return lookThroughWindows(brightGraySpotOut, brightGrayResult.maxLoc, pOutputFilenamePreamble);
    }

    // Look through the left and right windows and determine if the team prop
    // is in the left window, the right window, or neither. Also draw the boundaries
    // of the windows.
    private TeamPropReturn lookThroughWindows(Mat pPropOut, Point pCenterOfObject, String pOutputFilenamePreamble) {
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
        SpikeWindowUtils.drawSpikeWindows(pPropOut, spikeWindows, pOutputFilenamePreamble);

        return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, foundLocation);
    }

    // Split the original image ROI into its BGR channels. The alliance
    // determines which channel to pre-process and return. For better
    // contrast the RED alliance uses the inversion of the blue channel
    // and the BLUE alliance uses the inversion of the red channel.
    public static Mat splitAndInvertChannels(Mat pImageROI, RobotConstants.Alliance pAlliance, VisionParameters.GrayParameters pGrayParameters, String pOutputFilenamePreamble) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2
        Mat selectedChannel;
        switch (pAlliance) {
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
            default: throw new AutonomousRobotException(TAG, "Alliance must be RED or BLUE");
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

}