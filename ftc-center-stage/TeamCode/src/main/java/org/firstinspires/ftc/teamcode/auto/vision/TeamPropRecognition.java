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
                return redChannelCirclesPath(imageROI, outputFilenamePreamble, pTeamPropParameters.redChannelCirclesParameters);
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private TeamPropReturn redChannelCirclesPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                 TeamPropParameters.RedChannelCirclesParameters pRedChannelCirclesParameters) {

        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2
        Mat selectedChannel;
        switch (alliance) {
            case RED: {
                // Write out the red channel as grayscale.
                selectedChannel = channels.get(2);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", selectedChannel);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
                break;
            }
            // Note that the blue channel does not give great contrast
            // against the gray tiles. So use the inverted red channel.
            case BLUE: {
                // Write out the blue channel as grayscale.
                selectedChannel = channels.get(2); // channels.get(0);
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
                pRedChannelCirclesParameters.grayParameters.median_target);

        Mat readyForHoughCircles = new Mat();
        Imgproc.erode(adjustedGray, readyForHoughCircles, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(readyForHoughCircles, readyForHoughCircles, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Remove noise by Gaussian blurring.
        Imgproc.GaussianBlur(readyForHoughCircles, readyForHoughCircles, new Size(5, 5), 0);

        // Perform HoughCircles recognition
        Mat circles = new Mat();
        Imgproc.HoughCircles(readyForHoughCircles, circles, Imgproc.HOUGH_GRADIENT,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.dp,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.minDist,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.param1,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.param2,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.minRadius,
                pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.maxRadius);

        RobotLogCommon.d(TAG, "Number of circles " + circles.cols());

        // If no circles were found then assume that the prop is outside
        // of the ROI; use the NPOS position.
        Mat propOut = pImageROI.clone();
        if (circles.cols() == 0) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "No circles found; Team Prop location assumed as " + Objects.requireNonNull(nposWindow).second);
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
            if (radius < pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.minRadius) {
                // Circle is too small.
                RobotLogCommon.d(TAG, "False positive: circle too small, radius: " + radius);
                continue;
            }

            if (radius > pRedChannelCirclesParameters.houghCirclesFunctionCallParameters.maxRadius) {
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
            RobotLogCommon.d(TAG, "No circles passed the filters; Team Prop location assumed as " + Objects.requireNonNull(nposWindow).second);
            drawSpikeWindows(propOut, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, nposWindow.second);
        }

        // Draw a black circle at the center of the largest circle.
        Imgproc.circle(propOut, centerOfLargestCircle, 10, new Scalar(0, 0, 0), 4);

        String teamPropFilename = pOutputFilenamePreamble + "_CIR.png";
        RobotLogCommon.d(TAG, "Writing " + teamPropFilename);
        Imgcodecs.imwrite(teamPropFilename, propOut);
        RobotLogCommon.d(TAG, "Number of candidate team props found: " + numberOfTeamPropsFound);

        if (numberOfTeamPropsFound > pRedChannelCirclesParameters.maxCircles) {
            Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
            RobotLogCommon.d(TAG, "Number of circles (" + numberOfTeamPropsFound + ") " +
                    "exceeds the maximum of " + pRedChannelCirclesParameters.maxCircles);
            drawSpikeWindows(propOut, pOutputFilenamePreamble);
            return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL, Objects.requireNonNull(nposWindow).second);
        }

        return lookThroughWindows(propOut, centerOfLargestCircle, pOutputFilenamePreamble);
    }

    // Look through the left and right windows and determine if the team prop
    // is in the left window, the right window, or neither. Also draw the boundaries
    // of the windows.
    private TeamPropReturn lookThroughWindows(Mat pPropOut, Point pCenterOfLargestCircle,
                                              String pOutputFilenamePreamble) {
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> nposWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS);
        RobotConstantsCenterStage.TeamPropLocation foundLocation;

        if (leftWindow == null || rightWindow == null || nposWindow == null)
            throw new AutonomousRobotException(TAG, "Failed sanity check: no data for at least one of the spike windows");

        // Try the left window.
        if (pCenterOfLargestCircle.x >= leftWindow.first.x && pCenterOfLargestCircle.x < leftWindow.first.x + leftWindow.first.width) {
            RobotLogCommon.d(TAG, "Success: Team Prop found in the left spike window: location " + leftWindow.second);
            foundLocation = leftWindow.second;
        }
        else
            // Try the right window.
            if (pCenterOfLargestCircle.x >= rightWindow.first.x && pCenterOfLargestCircle.x < rightWindow.first.x + rightWindow.first.width) {
                RobotLogCommon.d(TAG, "Success: Team Prop found in the right spike window: location " + rightWindow.second);
                foundLocation = rightWindow.second;
            }
            else {
                RobotLogCommon.d(TAG, "Team Prop not found in the left or right window: assuming location " + nposWindow.second);
                foundLocation = nposWindow.second;
            }

        // Draw the spike windows on the ROI with the circles.
        drawSpikeWindows(pPropOut, pOutputFilenamePreamble);

        return new TeamPropReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, foundLocation);
    }

    private void drawSpikeWindows(Mat pPropOut, String pOutputFilenamePreamble) {
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow = spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);

        // Draw the spike windows on the ROI with the circles
        // so that we can see their placement during debugging.
        // params Mat, Point upperLeft, Point lowerRight, Scalar color, int thickness

        Point leftWindowUpperLeft = new Point(Objects.requireNonNull(leftWindow).first.x, leftWindow.first.y);
        Point leftWindowLowerRight = new Point(leftWindow.first.x + leftWindow.first.width,
                leftWindow.first.y + leftWindow.first.height);

        Point rightWindowUpperLeft = new Point(Objects.requireNonNull(rightWindow).first.x, rightWindow.first.y);
        Point rightWindowLowerRight = new Point(rightWindow.first.x + rightWindow.first.width,
                rightWindow.first.y + rightWindow.first.height);

        Imgproc.rectangle(pPropOut, leftWindowUpperLeft, leftWindowLowerRight, new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(pPropOut, rightWindowUpperLeft, rightWindowLowerRight, new Scalar(0, 255, 0), 3);

        String teamPropFilename = pOutputFilenamePreamble + "_PROP.png";
        RobotLogCommon.d(TAG, "Writing " + teamPropFilename);
        Imgcodecs.imwrite(teamPropFilename, pPropOut);
    }

}