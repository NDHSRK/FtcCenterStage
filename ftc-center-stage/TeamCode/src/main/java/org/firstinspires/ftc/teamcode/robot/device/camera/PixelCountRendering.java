package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.Objects;

public class PixelCountRendering implements CameraStreamRendering {
    private static final String TAG = PixelCountRendering.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final TeamPropParameters.ColorChannelPixelCountParameters pixelCountParameters;
    private final SpikeWindowMapping spikeWindowMapping;

    public PixelCountRendering(RobotConstants.Alliance pAlliance,
                               TeamPropParameters.ColorChannelPixelCountParameters pPixelCountParameters, SpikeWindowMapping pSpikeWindowMapping) {
        alliance = pAlliance;
        pixelCountParameters = pPixelCountParameters;
        spikeWindowMapping = pSpikeWindowMapping;
    }

    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {
        Pair<Mat, Date> teamPropImage = Pair.create(pWebcamFrame, new Date()); // for compatibility the the TeamPropRecognition workflow

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getDateTimeStamp(teamPropImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(spikeWindowMapping.imageParameters.image_source, WorkingDirectory.getWorkingDirectory(), fileDate);
        Mat imageROI = ImageUtils.preProcessImage(teamPropImage.first, outputFilenamePreamble, spikeWindowMapping.imageParameters);

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        int allianceMinWhitePixelCount;
        switch (alliance) {
            case RED: {
                allianceGrayParameters = pixelCountParameters.redGrayParameters;
                allianceMinWhitePixelCount = pixelCountParameters.redMinWhitePixelCount;
                break;
            }
            case BLUE: {
                allianceGrayParameters = pixelCountParameters.blueGrayParameters;
                allianceMinWhitePixelCount = pixelCountParameters.blueMinWhitePixelCount;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        // Threshold the image: set pixels over the threshold value to white.
        Mat split = TeamPropRecognition.splitAndInvertChannels(imageROI, alliance, allianceGrayParameters, outputFilenamePreamble);
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(split, thresholded,
                Math.abs(allianceGrayParameters.threshold_low),    // threshold value
                255,   // white
                allianceGrayParameters.threshold_low >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type
        RobotLogCommon.d(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        //**TODO show thresholded ROI on the DS camera stream.
    }

}
