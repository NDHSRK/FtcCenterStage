package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.auto.vision.ShapeDrawing;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class BackdropPixelRendering implements CameraStreamRendering {

    private final LinearOpMode linear;
    private final AtomicReference<VisionParameters.GrayParameters> backdropGrayParameters = new AtomicReference<>();
    private final VisionParameters.ImageParameters imageParameters;
    private final AtomicBoolean requestImageCapture = new AtomicBoolean();
    private int captureCount;
    private final String outputFilePreamble;
    private final Mat bgrFrame = new Mat();

    public BackdropPixelRendering(LinearOpMode pLinear,
                                  VisionParameters.ImageParameters pImageParameters,
                                  VisionParameters.GrayParameters pBackdropGrayParameters) {
        linear = pLinear;
        backdropGrayParameters.set(pBackdropGrayParameters);
        imageParameters = pImageParameters;
        outputFilePreamble = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
    }

    public void setGrayscaleThresholdParameters(VisionParameters.GrayParameters pGrayParameters) {
        backdropGrayParameters.set(pGrayParameters);
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

        if (captureNow) {
            String outputFilename = outputFilePreamble + "BackdropPixel_" + String.format(Locale.US, "_%04d_IMG.png", captureCount);
            Imgcodecs.imwrite(outputFilename, bgrFrame);
        }

        Mat imageROI = ImageUtils.preProcessImage(bgrFrame, null, imageParameters);
        VisionParameters.GrayParameters localGrayParameters = backdropGrayParameters.get();

        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(imageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // We're using the red channel.
        if (captureNow) {
            String outputFilename = outputFilePreamble + "BackdropPixel_" + String.format(Locale.US, "_%04d_RED.png", captureCount);
            Imgcodecs.imwrite(outputFilename, channels.get(2));
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), null, localGrayParameters.median_target, localGrayParameters.threshold_low);

        if (captureNow) {
            String outputFilename = outputFilePreamble + "BackdropPixel_" + String.format(Locale.US, "_%04d_THR.png", captureCount);
            Imgcodecs.imwrite(outputFilename, thresholded);
        }

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0)
            return;

        if (captureNow) {
            // Draw all of the contours.
            Mat contoursDrawn = bgrFrame.clone();
            ShapeDrawing.drawShapeContours(contours, contoursDrawn);
            String outputFilename = outputFilePreamble + "BackdropPixel_" + String.format(Locale.US, "_%04d_CON.png", captureCount);
            Imgcodecs.imwrite(outputFilename, contoursDrawn);
        }

        // Show the thresholded image in the DS camera stream.
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
        float insetLeft = (float) ((onscreenWidth / 2) - (imageParameters.image_roi.width / 2));
        float insetTop = (float) ((onscreenHeight / 2) - (imageParameters.image_roi.height / 2));
        pDriverStationScreenCanvas.drawBitmap(bmp, insetLeft, insetTop, null);

        //**TODO Draw a green boundary around the inset. See SpikeWindowRendering.
    }

}
