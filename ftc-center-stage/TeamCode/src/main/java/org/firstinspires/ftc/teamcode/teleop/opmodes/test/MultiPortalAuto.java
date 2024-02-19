package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// 10/20/2023 This class is a port of the sample MultiPortal
// from TeleOp to Autonomous.
public class MultiPortalAuto {

  private static final String TAG = MultiPortalAuto.class.getSimpleName();
  
  private final LinearOpMode linearOpMode;
  private final WebcamName webcam1Rear;
  private final WebcamName webcam2Front;

  VisionPortal.Builder myVisionPortalBuilder;
  boolean USE_WEBCAM_1;
  int Portal_1_View_ID;
  boolean USE_WEBCAM_2;
  int Portal_2_View_ID;
  AprilTagProcessor myAprilTagProcessor_1;
  AprilTagProcessor myAprilTagProcessor_2;
  //WebcamFrameProcessor webcamFrameProcessor; //##PY added 10/5/23

  VisionPortal myVisionPortal_1;
  VisionPortal myVisionPortal_2;
  
     public MultiPortalAuto(LinearOpMode pLinearOpMode, WebcamName pWebcam1Rear, WebcamName pWebcam2Front) {

        RobotLogCommon.c(TAG, "MultiPortalAuto constructor");

        linearOpMode = pLinearOpMode; // FTC context
       webcam1Rear = pWebcam1Rear;
       webcam2Front = pWebcam2Front;

       // This OpMode shows AprilTag recognition and pose estimation.
       USE_WEBCAM_1 = true;
       USE_WEBCAM_2 = true;
       //##PY 10/20/2023 run without view ids initMultiPortals();

       //##PY added 10/5/23
       //webcamFrameProcessor = new WebcamFrameProcessor.Builder().build();

       // Initialize AprilTag before waitForStart.
       initAprilTag();
       
       // Wait for the Start button to be touched.
       //##PY 10/20/2023 In Autonomous we can't call waitForStart() so we'll
       // just delay a couple of seconds before exiting the constructor.
       linearOpMode.telemetry.addLine("Waiting 2 seconds to start");
       linearOpMode.telemetry.update();
       //waitForStart();
       sleepInLoop(2000);
        }
        
  /**
   * This Op Mode demonstrates MultiPortalView.
   *
   * The Dpad buttons can turn each camera stream on and off.
   * USB bandwidth is more restricted on an external USB hub, compared to the Control Hub.
   */
  //##PY keep the method name even though we're not a LinearOpMode in TeleOp.
  public void runOpMode() {

    //##PY added 10/5/23 for webcam frames int webcamFrameCount = 1;
    if (linearOpMode.opModeIsActive()) {
      while (linearOpMode.opModeIsActive()) {
        AprilTag_telemetry_for_Portal_1();

        //##PY added 10/5/23 add telemetry for webcam frames
        //Pair<Mat, Date> frameVal = webcamFrameProcessor.getWebcamFrame();
        //linearOpMode.telemetry.addLine("Received raw webcam frame " + webcamFrameCount++);

        AprilTag_telemetry_for_Portal_2();
        AprilTag_telemetry_legend();
        Toggle_camera_streams();
        // Push telemetry to the Driver Station.
        linearOpMode.telemetry.update();
        // Share the CPU.
        linearOpMode.sleep(20);
      }
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;

    // First, create an AprilTagProcessor.Builder.
    /*
    Logitech Brio - calibrated
             <focal_length_x>627.419488832</focal_length_x>
          <focal_length_y>627.419488832</focal_length_y>
          <optical_center_x>301.424062225</optical_center_x>
          <optical_center_y>234.042415697</optical_center_y>
     */
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    myAprilTagProcessorBuilder.setLensIntrinsics(627.420, 627.420, 301.424, 234.0424);

    // Create each AprilTagProcessor by calling build.
    myAprilTagProcessor_1 = myAprilTagProcessorBuilder.build();
    myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
    Make_first_VisionPortal();
    Make_second_VisionPortal();
  }

  /**
   * Describe this function...
   */
  private void Make_first_VisionPortal() {
    // Create a VisionPortal.Builder and set attributes related to the first camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM_1) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(webcam1Rear);
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(640, 480)); //##PY use full default resolution
    // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor_1);
    // Add the Portal View ID to the VisionPortal.Builder

    //##PY 10/5/23 do not use live view
    myVisionPortalBuilder.enableLiveView(false); //##PY changed to false 10/5/23
    // Set the camera monitor view id.
    //myVisionPortalBuilder.setLiveViewContainerId(Portal_1_View_ID);

    // Create a VisionPortal by calling build.
    myVisionPortal_1 = myVisionPortalBuilder.build();
  }

  /**
   * Describe this function...
   */
  private void Make_second_VisionPortal() {
    if (USE_WEBCAM_2) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(webcam2Front);
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(640, 480)); //##PY use full default resolution
    // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

    //##PY added 10/5/23
    //myVisionPortalBuilder.addProcessor(webcamFrameProcessor);

    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor_2);

    // Add the Portal View ID to the VisionPortal.Builder

    //##PY 10/5/23 do not use live view
    myVisionPortalBuilder.enableLiveView(false); //##PY changed to false 10/5/23
    // Set the camera monitor view id.
    //myVisionPortalBuilder.setLiveViewContainerId(Portal_2_View_ID);

    // Create a VisionPortal by calling build.
    myVisionPortal_2 = myVisionPortalBuilder.build();
  }

  /**
   * Describe this function...
   */
  private void Toggle_camera_streams() {
    // Manage USB bandwidth of two camera streams, by turning on or off.
    if (linearOpMode.gamepad1.dpad_down) {
      // Temporarily stop the streaming session. This can save CPU
      // resources, with the ability to resume quickly when needed.
      myVisionPortal_1.stopStreaming();
      linearOpMode.telemetry.addLine("Webcam 1 (rear) stop streaming");
      linearOpMode.telemetry.update();
    } else if (linearOpMode.gamepad1.dpad_up) {
      // Resume the streaming session if previously stopped.
      myVisionPortal_1.resumeStreaming();
      linearOpMode.telemetry.addLine("Webcam 1 (rear) resume streaming");
      linearOpMode.telemetry.update();
    }
    if (linearOpMode.gamepad1.dpad_left) {
      // Temporarily stop the streaming session. This can save CPU
      // resources, with the ability to resume quickly when needed.
      myVisionPortal_2.stopStreaming();
      linearOpMode.telemetry.addLine("Webcam 2 (front) stop streaming");
      linearOpMode.telemetry.update();
    } else if (linearOpMode.gamepad1.dpad_right) {
      // Resume the streaming session if previously stopped.
      myVisionPortal_2.resumeStreaming();
      linearOpMode.telemetry.addLine("Webcam 2 (front) resume streaming");
      linearOpMode.telemetry.update();
    }
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void AprilTag_telemetry_for_Portal_1() {
    List<AprilTagDetection> myAprilTagDetections_1;
    AprilTagDetection thisDetection_1;

    // Get a list of AprilTag detections.
    myAprilTagDetections_1 = myAprilTagProcessor_1.getDetections();

    linearOpMode.telemetry.addData("Portal 1 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_1));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection thisDetection_1_item : myAprilTagDetections_1) {
      thisDetection_1 = thisDetection_1_item;
      // Display info about the detection.
      linearOpMode.telemetry.addLine("");
      if (thisDetection_1.metadata != null) {
        linearOpMode.telemetry.addLine("==== (ID " + thisDetection_1.id + ") " + thisDetection_1.metadata.name);
        linearOpMode.telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_1.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.z, 6, 1) + "  (inch)");
        linearOpMode.telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_1.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.roll, 6, 1) + "  (deg)");
        linearOpMode.telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_1.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        linearOpMode.telemetry.addLine("==== (ID " + thisDetection_1.id + ") Unknown");
        linearOpMode.telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_1.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_1.center.y, 6, 0) + " (pixels)");
      }
    }
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void AprilTag_telemetry_for_Portal_2() {
    List<AprilTagDetection> myAprilTagDetections_2;
    AprilTagDetection thisDetection_2;

    // Get a list of AprilTag detections.
    myAprilTagDetections_2 = myAprilTagProcessor_2.getDetections();

    linearOpMode.telemetry.addLine("");
    linearOpMode.telemetry.addData("Portal 2 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_2));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection thisDetection_2_item : myAprilTagDetections_2) {
      thisDetection_2 = thisDetection_2_item;
      // Display info about the detection.
      linearOpMode.telemetry.addLine("");
      if (thisDetection_2.metadata != null) {
        linearOpMode.telemetry.addLine("==== (ID " + thisDetection_2.id + ") " + thisDetection_2.metadata.name);
        linearOpMode.telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_2.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.z, 6, 1) + "  (inch)");
        linearOpMode.telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_2.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.roll, 6, 1) + "  (deg)");
        linearOpMode.telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_2.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        linearOpMode.telemetry.addLine("==== (ID " + thisDetection_2.id + ") Unknown");
        linearOpMode.telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_2.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_2.center.y, 6, 0) + " (pixels)");
      }
    }
  }

  /**
   * Describe this function...
   */
  private void AprilTag_telemetry_legend() {
    linearOpMode.telemetry.addLine("");
    linearOpMode.telemetry.addLine("key:");
    linearOpMode.telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    linearOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    linearOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

  // Sleeps but also tests if the OpMode is still active.
  // Telemetry should keep the FTC runtime from shutting us down due to inactivity.
  private void sleepInLoop(int pMilliseconds) {
    RobotLogCommon.d(TAG, "Sleep for " + pMilliseconds + " milliseconds");
    linearOpMode.telemetry.addData("Sleeping for: ", "%d", pMilliseconds);
    linearOpMode.telemetry.update();

    int numSleeps = pMilliseconds / 100;
    int sleepRemainder = pMilliseconds % 100;
    int msSlept = 0;
    for (int i = 0; i < numSleeps; i++) {
      if (!linearOpMode.opModeIsActive())
        return;
      linearOpMode.sleep(100);
      msSlept += 100;

      // Only update telemetry every 500ms
      if (msSlept % 500 == 0) {
        linearOpMode.telemetry.addData("Slept for: ", "%d", msSlept);
        linearOpMode.telemetry.update();
      }
    }

    if (linearOpMode.opModeIsActive() && sleepRemainder != 0)
      linearOpMode.sleep(sleepRemainder);
  }
  
}
