package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.Pair;
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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class PixelCountRendering implements CameraStreamRendering {
    private static final String TAG = PixelCountRendering.class.getSimpleName();

    private final LinearOpMode linear;
    private final RobotConstantsCenterStage.OpMode opMode;
    private final RobotConstants.Alliance alliance;
    private final AtomicReference<VisionParameters.GrayParameters> allianceGrayParameters = new AtomicReference<>();
    private final int allianceMinWhitePixelCount;
    private final SpikeWindowMapping spikeWindowMapping;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow;
    private final AtomicBoolean requestImageCapture = new AtomicBoolean();
    private int captureCount;
    private final String outputFilePreamble;
    private Mat bgrFrame = new Mat();

    public PixelCountRendering(LinearOpMode pLinear, RobotConstantsCenterStage.OpMode pOpMode,
                               RobotConstants.Alliance pAlliance,
                               VisionParameters.GrayParameters pAllianceGrayParameters,
                               int pAllianceMinWhitePixelCount,
                               SpikeWindowMapping pSpikeWindowMapping) {
        linear = pLinear;
        opMode = pOpMode;
        alliance = pAlliance;
        allianceGrayParameters.set(pAllianceGrayParameters);
        allianceMinWhitePixelCount = pAllianceMinWhitePixelCount;
        spikeWindowMapping = pSpikeWindowMapping;
        leftWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        rightWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);
        outputFilePreamble = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
    }

    public void setGrayscaleThresholdParameters(VisionParameters.GrayParameters pGrayParameters) {
        allianceGrayParameters.set(pGrayParameters);
    }

    public void requestImageCapture() {
        requestImageCapture.set(true);
    }

    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {
        boolean captureNow = requestImageCapture.getAndSet(false);
        if (captureNow)
            captureCount++;

        Imgproc.cvtColor(pWebcamFrame, bgrFrame, Imgproc.COLOR_RGBA2BGR);
        Mat imageROI = ImageUtils.preProcessImage(bgrFrame, null, spikeWindowMapping.imageParameters);

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters localGrayParameters = allianceGrayParameters.get();
        Mat split = TeamPropRecognition.splitAndInvertChannels(imageROI, alliance, localGrayParameters, null);

        if (captureNow) {
            String outputFilename = outputFilePreamble + "PixelCount_" + opMode + String.format(Locale.US, "_%04d_INV.png", captureCount);
            Imgcodecs.imwrite(outputFilename, split);
        }

        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(split, thresholded,
                Math.abs(localGrayParameters.threshold_low),    // threshold value
                255,   // white
                localGrayParameters.threshold_low >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type

        if (captureNow) {
            String outputFilename = outputFilePreamble + "PixelCount_" + opMode + String.format(Locale.US, "_%04d_THR.png", captureCount);
            Imgcodecs.imwrite(outputFilename, thresholded);
        }

        linear.telemetry.addLine("Grayscale median " + localGrayParameters.median_target);
        linear.telemetry.addLine("Grayscale low threshold " + localGrayParameters.threshold_low);
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
        linear.telemetry.update();

        // Show the thresholded ROI in the DS camera stream.
        // First convert the thresholded ROI to an Android Bitmap.
        // See https://stackoverflow.com/questions/44579822/convert-opencv-mat-to-android-bitmap
        Bitmap bmp = Bitmap.createBitmap(thresholded.cols(), thresholded.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(thresholded, bmp);

        // What to do about scaling the thresholded Bitmap for display on the
        // Canvas? This is not so easy because of the odd ROI sizes. So for a
        // Canvas (onscreenWidth, onscreenHeight) of 960x720, an ROI of 492x259
        // produces a vertically elongated image. Scaling in Gimp produces an image
        // of 960x466. So we can either live with the elongated image - because it
        // does show the thresholding - or use an inset that is the size of the ROI.

        // The load the Bitmap onto the Canvas.
        // See https://stackoverflow.com/questions/30630887/android-bitmap-on-canvas-from-external-file
        /*
          This does it canvas.drawBitmap(bitmap,null, new Rect(a,120,a+200,270), null); made the source rect null
          Mark Barr
          Jun 4, 2015 at 17:38
         */

        //## Implicit scaling - where you use bmp as the bitmap - shows the same elongated image.
        //android.graphics.Rect destRect = new android.graphics.Rect(0, 0, onscreenWidth, onscreenHeight);
        //pDriverStationScreenCanvas.drawBitmap(bmp, null, destRect, null);

        // This method displays a centered inset.
        float insetLeft = (onscreenWidth / 2) - (spikeWindowMapping.imageParameters.image_roi.width / 2);
        float insetTop = (onscreenHeight / 2) - (spikeWindowMapping.imageParameters.image_roi.height / 2);
        pDriverStationScreenCanvas.drawBitmap(bmp, insetLeft, insetTop, null);
    }

}
