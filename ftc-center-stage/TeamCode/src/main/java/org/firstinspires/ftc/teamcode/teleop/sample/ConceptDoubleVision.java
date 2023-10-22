/* Copyright (c) 2023 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop.sample;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.device.camera.WebcamFrameProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.Date;
import java.util.List;

// This OpMode is based on the sample ConceptDoubleVision - but instead of
// TensorFlow Object Detection it uses our custom processor for raw frames:
// WebcamFrameProcessor.

@TeleOp(name = "Concept: Double Vision", group = "Concept")
//@Disabled
public class ConceptDoubleVision extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the WebcamFrameProcessor.
     */
    private WebcamFrameProcessor webcamFrame;
    private int framesCaptured = 0;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        initDoubleVision();

        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and WebcamFrame processors.
        while (!isStopRequested()) {

            if (opModeInInit()) {
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
            }

            if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                // User instructions: Dpad left or Dpad right.
                telemetry.addLine("Dpad Left to disable AprilTag");
                telemetry.addLine();
                telemetryAprilTag();
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag");
            }
            telemetry.addLine();
            telemetry.addLine("----------------------------------------");
            if (myVisionPortal.getProcessorEnabled(webcamFrame)) {
                telemetry.addLine("Dpad Down to disable raw frames");
                telemetry.addLine();
                telemetryWebcamFrame();
            } else {
                telemetry.addLine("Dpad Up to enable raw frames");
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            if (gamepad1.dpad_left) {
                myVisionPortal.setProcessorEnabled(aprilTag, false);
            } else if (gamepad1.dpad_right) {
                myVisionPortal.setProcessorEnabled(aprilTag, true);
            }
            if (gamepad1.dpad_down) {
                myVisionPortal.setProcessorEnabled(webcamFrame, false);
            } else if (gamepad1.dpad_up) {
                myVisionPortal.setProcessorEnabled(webcamFrame, true);
            }

            Toggle_camera_stream();

            sleep(20);

        }   // end while loop

    }   // end method runOpMode()


    /**
     * Initialize AprilTag and WebcamFrame.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                //##PY for Logitech StreamCam
                .setLensIntrinsics(622.001, 622.001, 319.803, 241)
                // ##PY for Logitech Brio from the 3DF Zephyr tool
                //.setLensIntrinsics(627.419488832, 627.419488832, 301.424062225, 234.042415697)
                .build();

        // -----------------------------------------------------------------------------------------
        // WebcamFrame Configuration
        // -----------------------------------------------------------------------------------------

        webcamFrame = new WebcamFrameProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(webcamFrame)
                .addProcessor(aprilTag)
                .build();

    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about WebcamFrame recognitions.
     */
    private void telemetryWebcamFrame() {
        Pair<Mat, Date> frame = webcamFrame.getWebcamFrame();
        if (frame == null)
            telemetry.addLine("No frame captured");
        else
            telemetry.addLine("Frames captured " + ++framesCaptured);

    }   // end method telemetryWebcamFrame()

     private void Toggle_camera_stream() {
         // Manage USB bandwidth of two camera streams, by turning on or off.
         if (gamepad1.a) {
             // Temporarily stop the streaming session. This can save CPU
             // resources, with the ability to resume quickly when needed.
             myVisionPortal.stopStreaming();
             telemetry.addLine("Stop streaming");
             telemetry.update();
         } else if (gamepad1.y) {
             // Resume the streaming session if previously stopped.
             myVisionPortal.resumeStreaming();
             telemetry.addLine("Resume streaming");
             telemetry.update();
         }
     }

}   // end class
