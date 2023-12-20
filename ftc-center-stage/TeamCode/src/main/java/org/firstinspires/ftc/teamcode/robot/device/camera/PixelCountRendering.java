package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PixelCountRendering implements CameraStreamRendering {
    private static final String TAG = PixelCountRendering.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final AtomicReference<VisionParameters.GrayParameters> allianceGrayParameters = new AtomicReference<>();
    private final int allianceMinWhitePixelCount;
    private final SpikeWindowMapping spikeWindowMapping;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow;
    private final Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow;

    public PixelCountRendering(RobotConstants.Alliance pAlliance,
                               TeamPropParameters.ColorChannelPixelCountParameters pPixelCountParameters, SpikeWindowMapping pSpikeWindowMapping) {
        alliance = pAlliance;
        spikeWindowMapping = pSpikeWindowMapping;
        leftWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        rightWindow = spikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);

        // Set the initial values for grayscale thresholding.
        if (alliance == RobotConstants.Alliance.BLUE) {
            allianceGrayParameters.set(pPixelCountParameters.blueGrayParameters);
            allianceMinWhitePixelCount = pPixelCountParameters.blueMinWhitePixelCount;
        }
        else {
            allianceGrayParameters.set(pPixelCountParameters.redGrayParameters);
            allianceMinWhitePixelCount = pPixelCountParameters.redMinWhitePixelCount;
        }
    }

    public void setGrayscaleThresholdParameters(VisionParameters.GrayParameters pGrayParameters) {
        allianceGrayParameters.set(pGrayParameters);
    }

    public List<String> renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {
        List<String> telemetryLines = new ArrayList<>();

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

        telemetryLines.add("Minimum white pixel count " + allianceMinWhitePixelCount);
        telemetryLines.add("Threshold values: low " + localGrayParameters.threshold_low + ", high 255");

        // Get the white pixel count for both the left and right
        // spike windows.
        Rect leftSpikeWindowBoundary = leftWindow.first;
        Mat leftSpikeWindow = thresholded.submat(leftSpikeWindowBoundary);
        int leftNonZeroCount = Core.countNonZero(leftSpikeWindow);
        telemetryLines.add(leftWindow.second.toString() + " white pixel count " + leftNonZeroCount);

        Rect rightSpikeWindowBoundary = rightWindow.first;
        Mat rightSpikeWindow = thresholded.submat(rightSpikeWindowBoundary);
        int rightNonZeroCount = Core.countNonZero(rightSpikeWindow);
        telemetryLines.add(rightWindow.second.toString() + " white pixel count " + rightNonZeroCount);

        //**TODO show thresholded ROI on the DS camera stream.
        // See https://stackoverflow.com/questions/30630887/android-bitmap-on-canvas-from-external-file
        // but I should be able to use my ROI width and height and the parameters int onscreenWidth,
        // int onscreenHeight for the source and destination Android Rects.
        /*

This does it canvas.drawBitmap(bitmap,null, new Rect(a,120,a+200,270), null); made the source rect null
Mark Barr
Jun 4, 2015 at 17:38

    // From https://stackoverflow.com/questions/44579822/convert-opencv-mat-to-android-bitmap
    // ARGB_8888 works for grayscale also.
    private static Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = null;
        Mat rgb = new Mat();
        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_BGR2RGB);

        try {
            bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgb, bmp);
        }
        catch (CvException e){
            Log.d("Exception",e.getMessage());
        }
        return bmp;
    }

    // The above should work according to the documentation:
     public static void matToBitmap(Mat mat,
                               android.graphics.Bitmap bmp)

Short form of the matToBitmap(mat, bmp, premultiplyAlpha=false)
    mat - is a valid input Mat object of the types 'CV_8UC1', 'CV_8UC3' or 'CV_8UC4'.
    bmp - is a valid Bitmap object of the same size as the Mat and of type 'ARGB_8888' or 'RGB_565'.
     */

        return telemetryLines;
    }

}
