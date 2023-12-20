package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.concurrent.atomic.AtomicReference;

public class PixelCountRendering implements CameraStreamRendering {
    private static final String TAG = PixelCountRendering.class.getSimpleName();

    private final LinearOpMode linear;
    private final RobotConstants.Alliance alliance;
    private final AtomicReference<VisionParameters.GrayParameters> allianceGrayParameters = new AtomicReference<>();
    private final int allianceMinWhitePixelCount;
    private final SpikeWindowMapping spikeWindowMapping;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow;

    public PixelCountRendering(LinearOpMode pLinear, RobotConstants.Alliance pAlliance,
                               VisionParameters.GrayParameters pAllianceGrayParameters,
                               int pAllianceMinWhitePixelCount,
                               SpikeWindowMapping pSpikeWindowMapping) {
        linear = pLinear;
        alliance = pAlliance;
        spikeWindowMapping = pSpikeWindowMapping;
        leftWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        rightWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);
        allianceGrayParameters.set(pAllianceGrayParameters);
        allianceMinWhitePixelCount = pAllianceMinWhitePixelCount;
    }

    public void setGrayscaleThresholdParameters(VisionParameters.GrayParameters pGrayParameters) {
        allianceGrayParameters.set(pGrayParameters);
    }

    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {
        Pair<Mat, Date> teamPropImage = Pair.create(pWebcamFrame, new Date()); // for compatibility the the TeamPropRecognition workflow
        String fileDate = TimeStamp.getDateTimeStamp(teamPropImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(spikeWindowMapping.imageParameters.image_source, WorkingDirectory.getWorkingDirectory(), fileDate);
        Mat imageROI = ImageUtils.preProcessImage(teamPropImage.first, outputFilenamePreamble, spikeWindowMapping.imageParameters);

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters localGrayParameters = allianceGrayParameters.get();
        Mat split = TeamPropRecognition.splitAndInvertChannels(imageROI, alliance, localGrayParameters, outputFilenamePreamble);
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(split, thresholded,
                Math.abs(localGrayParameters.threshold_low),    // threshold value
                255,   // white
                localGrayParameters.threshold_low >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type

        linear.telemetry.addLine("Minimum white pixel count " + allianceMinWhitePixelCount);
        linear.telemetry.addLine("Threshold values: low " + localGrayParameters.threshold_low + ", high 255");

        // Get the white pixel count for both the left and right
        // spike windows.
        Rect leftSpikeWindowBoundary = leftWindow.first;
        Mat leftSpikeWindow = thresholded.submat(leftSpikeWindowBoundary);
        int leftNonZeroCount = Core.countNonZero(leftSpikeWindow);
        linear.telemetry.addLine(leftWindow.second.toString() + " white pixel count " + leftNonZeroCount);

        Rect rightSpikeWindowBoundary = rightWindow.first;
        Mat rightSpikeWindow = thresholded.submat(rightSpikeWindowBoundary);
        int rightNonZeroCount = Core.countNonZero(rightSpikeWindow);
        linear.telemetry.addLine(rightWindow.second.toString() + " white pixel count " + rightNonZeroCount);

        // Show the thresholded ROI in the DS camera stream.
        // First convert the thresholded ROI to an Android Bitmap.
        // See https://stackoverflow.com/questions/44579822/convert-opencv-mat-to-android-bitmap
        // ARGB_8888 works for grayscale also.
        Bitmap bmp = Bitmap.createBitmap(thresholded.cols(), thresholded.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(thresholded, bmp);

        // The load the Bitmap onto the Canvas.
        // See https://stackoverflow.com/questions/30630887/android-bitmap-on-canvas-from-external-file
        /*
          This does it canvas.drawBitmap(bitmap,null, new Rect(a,120,a+200,270), null); made the source rect null
          Mark Barr
          Jun 4, 2015 at 17:38
         */
        android.graphics.Rect sourceRect = new android.graphics.Rect(0, 0, thresholded.cols(), thresholded.rows());
        android.graphics.Rect destRect = new android.graphics.Rect(0, 0, onscreenWidth, onscreenHeight);
    }

}
