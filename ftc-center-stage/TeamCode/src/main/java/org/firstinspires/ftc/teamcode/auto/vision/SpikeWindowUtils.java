package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.robot.device.camera.ImageProvider;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.EnumMap;

public class SpikeWindowUtils {

    private static final String TAG = SpikeWindowUtils.class.getSimpleName();

    private final String imageDirectory;

    public SpikeWindowUtils() {
        imageDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
    }

    // Returns the ROI from the full image with the spike windows
    // drawn in.
    public Mat overlaySpikeWindows(ImageProvider pImageProvider,
                                            String pImageFilename,
                                            SpikeWindowMapping pSpikeWindowMapping) throws InterruptedException {
        RobotLogCommon.d(TAG, "In " + TAG + ". showSpikeWindowLayout");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> teamPropImage = pImageProvider.getImage();
        if (teamPropImage == null)
            return null; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getDateTimeStamp(teamPropImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageFilename, imageDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(teamPropImage.first, outputFilenamePreamble, pSpikeWindowMapping.imageParameters);
        drawSpikeWindows(imageROI, pSpikeWindowMapping.spikeWindows, outputFilenamePreamble);
        return imageROI;
    }

    public static void drawSpikeWindows(Mat pPropOut,
                                  EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> pSpikeWindows,
                                  String pOutputFilenamePreamble) {
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = pSpikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> rightWindow = pSpikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT);

        // Draw the spike windows on the ROI
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

        if (pOutputFilenamePreamble != null) {
            String teamPropFilename = pOutputFilenamePreamble + "_SPIKE.png";
            RobotLogCommon.d(TAG, "Writing " + teamPropFilename);
            Imgcodecs.imwrite(teamPropFilename, pPropOut);
        }
    }

}