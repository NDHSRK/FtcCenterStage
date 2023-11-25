/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//##PY 08/31/2023 This class is derived from the FTC 8.2 April Tag
// implementation AprilTagProcessorImpl.java.

package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.vision.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.concurrent.atomic.AtomicReference;

// Follow the class relationships of the AprilTag sample in which
// AprilTagProcessorImpl inherits from AprilTagProcessorImpl.
public class SpikeWindowProcessorImpl extends SpikeWindowProcessor {

    private final String TAG = SpikeWindowProcessorImpl.class.getSimpleName();

    private final AtomicReference<SpikeWindowMapping> spikeWindowMapping = new AtomicReference<>();

    //## This is a callback. It definitely runs on another thread.
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Init is called from VisionPortalImpl when the first frame for this
        // processor has been received; the frame itself is not passed in
        // here.
    }

    //## This is a callback; assume it's running on another thread.
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // If no spike window mapping has been set yet, just return
        // the input.
        SpikeWindowMapping currentSpikeWindowMapping = spikeWindowMapping.get();
        if (currentSpikeWindowMapping == null)
            return input;

        // Now we have a spike window mapping. Create the required UserData
        // for the onDrawFrame callback.
        Pair<Rect, RobotConstantsCenterStage.TeamPropLocation> leftWindow = currentSpikeWindowMapping.spikeWindows.get(RobotConstantsCenterStage.SpikeLocationWindow.LEFT);
        return new SpikeWindowUserContext(currentSpikeWindowMapping.imageParameters.resolution_width,
                currentSpikeWindowMapping.imageParameters.resolution_height,
                currentSpikeWindowMapping.imageParameters.image_roi,
                leftWindow.first);
    }

    //## This is a callback.
    // For 640x480 full image the first callback to this method sent in the
    // following arguments:
    // [2023-11-24 06:48:30.111] [FINE   ] SpikeWindowProcessorImpl Canvas width 960, height 720
    // [2023-11-24 06:48:30.113] [FINE   ] SpikeWindowProcessorImpl Scale x 1.5, density 1.0
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Default path if no spike window has been set - this method will receive
        // a Mat as its userContext.
        if (userContext instanceof Mat)
            return; // We have nothing to add

        SpikeWindowUserContext localContext = (SpikeWindowUserContext) userContext;

        float xFactor = onscreenWidth / (float) localContext.resolutionWidth;
        float yFactor = onscreenHeight / (float) localContext.resolutionHeight;

        // Draw the ROI on the canvas.
        Paint greenAxisPaint = new Paint();
        greenAxisPaint.setColor(Color.GREEN);
        greenAxisPaint.setAntiAlias(true);
        greenAxisPaint.setStyle(Paint.Style.STROKE);
        greenAxisPaint.setStrokeCap(Paint.Cap.BUTT);
        greenAxisPaint.setStrokeWidth(8);

        float left = localContext.roiRect.x * xFactor;
        float top = localContext.roiRect.y * yFactor;
        float right = left + (localContext.roiRect.width * xFactor);
        float bottom = top + (localContext.roiRect.height * yFactor);
        canvas.drawRect(left, top, right, bottom, greenAxisPaint);

        // Draw the vertical line that separates the left and right
        // spike windows.
        float spikeWindowBoundaryX = left + (localContext.leftWindow.width * xFactor);
        canvas.drawLine(spikeWindowBoundaryX, top, spikeWindowBoundaryX, bottom, greenAxisPaint);
    }

    @Override
    public void setSpikeWindowMapping(SpikeWindowMapping pSpikeWindowMapping) {
        spikeWindowMapping.set(pSpikeWindowMapping);
    }

    private static class SpikeWindowUserContext {
        private final int resolutionWidth;
        private final int resolutionHeight;
        private final Rect roiRect;
        private final Rect leftWindow;

        private SpikeWindowUserContext(int pResolutionWidth, int pResolutionHeight,
                                       Rect pROIRect, Rect pLeftWindow) {
            resolutionWidth = pResolutionWidth;
            resolutionHeight = pResolutionHeight;
            roiRect = pROIRect;
            leftWindow = pLeftWindow;
        }
    }

}