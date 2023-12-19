package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.Objects;

public class PixelCountRendering implements CameraStreamRendering {

    private final SpikeWindowMapping spikeWindowMapping;

    public PixelCountRendering(SpikeWindowMapping pSpikeWindowMapping) {
        spikeWindowMapping = pSpikeWindowMapping;
    }

    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {
 //**TODO render thresholded pixel count grayscale ROI to Canvas.
    }

}
