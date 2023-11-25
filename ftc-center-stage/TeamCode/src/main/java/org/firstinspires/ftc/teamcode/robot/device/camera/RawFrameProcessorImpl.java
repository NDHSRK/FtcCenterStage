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

import org.firstinspires.ftc.ftcdevcommon.Pair;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.concurrent.atomic.AtomicReference;

// Follow the class relationships of the AprilTag sample in which
// AprilTagProcessorImpl inherits from AprilTagProcessorImpl.
public class RawFrameProcessorImpl extends RawFrameProcessor {
    private static final String TAG = RawFrameProcessorImpl.class.getSimpleName();

    Mat bgrFrame = new Mat();
    private final AtomicReference<Pair<Mat, Date>> webcamFrame = new AtomicReference<>();

    //## This is a callback. It definitely runs on another thread.
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
       // Init is called from VisionPortalImpl when the first frame for this
        // processor has been received; the frame itself is not passed in
        // here. It would be nice to use a CountDownLatch to let the main
        // thread know that the first frame has been received. But this leads
        // to an inconsistency because the provided AprilTagProcessorImpl does
        // not provide any notifications in its init() and we'd have to change
        // the source code, which is illegal, to access a CountDownLatch there.
    }
 
    //## This is a callback; assume it's running on another thread.
    // So store the frame in an AtomicReference.
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // From the EasyOpenCV readme:
        // **IMPORTANT NOTE:** EasyOpenCV delivers RGBA frames
        // So we need to convert to BGR for OpenCV here.
        Imgproc.cvtColor(input, bgrFrame, Imgproc.COLOR_RGBA2BGR);
        webcamFrame.set(Pair.create(bgrFrame, new Date()));
        return input;
    }

    //## This is a callback.
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //## If you don't put in some work to draw on the Canvas then
        // in the Driver Station camera stream you will only see the
        // original image.
    }

    @Override
    public Pair<Mat, Date> getWebcamFrame() {
            return webcamFrame.getAndSet(null);
    }
}