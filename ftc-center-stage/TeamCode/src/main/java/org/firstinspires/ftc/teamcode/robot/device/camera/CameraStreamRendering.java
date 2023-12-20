package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.opencv.core.Mat;

import java.util.Date;
import java.util.List;

public interface CameraStreamRendering {

    // Receives a webcam frame as input and renders it to the Canvas
    // for display on the Driver Station screen. Returns a collection
    // of lines for display in telemetry.
    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                            int onscreenWidth, int onscreenHeight);
}
