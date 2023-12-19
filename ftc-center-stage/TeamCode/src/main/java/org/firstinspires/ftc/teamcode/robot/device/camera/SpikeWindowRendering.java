package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.opencv.core.Mat;

public class SpikeWindowRendering implements CameraStreamRendering {

    private final SpikeWindowMapping spikeWindowMapping;
    public SpikeWindowRendering(SpikeWindowMapping pSpikeWindowMapping) {
        spikeWindowMapping = pSpikeWindowMapping;
    }

    public void renderFrameToCanvas(Mat pWebcamFrame, Canvas pDriverStationScreenCanvas,
                                    int onscreenWidth, int onscreenHeight) {

    };
}
