
package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

// Use the VisionPortal API to manage a webcam and either one or two
// "processors": one for raw webcam frames and the other for AprilTags.
//
// It is an application-wide assumption that there is only one instance of
// this class and that its methods are called from a single thread.
public class VisionPortalWebcam {
    private static final String TAG = VisionPortalWebcam.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    private final WebcamFrameProcessor webcamFrameProcessor;
    private final AprilTagProcessor aprilTagProcessor;
    private VisionProcessor activeProcessor;
    private RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId;
    private final VisionPortal visionPortal;

    public VisionPortalWebcam(FTCRobot pRobot) {
        //**TODO for now support only the Team Prop/AprilTag camera and a single processor.
        // Note: even if we attach two processors, only one at a time will be enabled.
        // See the sample ConceptDoubleVision.
        configuredWebcam = pRobot.visionPortalWebcamConfiguration.webcams.get(0);
        switch (configuredWebcam.processorId) {
            case WEBCAM_FRAME: {
                webcamFrameProcessor = new WebcamFrameProcessor.Builder().build();
                activeProcessor = webcamFrameProcessor;
                aprilTagProcessor = null;
                break;
            }
            case APRIL_TAG: {
                if (configuredWebcam.cameraCalibration == null)
                    throw new AutonomousRobotException(TAG, "Missing webcam calibration for AprilTag processor");

                aprilTagProcessor = new AprilTagProcessor.Builder()
                        //.setDrawAxes(false)
                        //.setDrawCubeProjection(false)
                        .setDrawTagOutline(true)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        //##PY .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                        // == CAMERA CALIBRATION ==
                        // If you do not manually specify calibration parameters, the SDK will attempt
                        // to load a predefined calibration for your camera.
                        // ... these parameters are fx, fy, cx, cy.
                        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                        // ##PY for Logitech Brio from the 3DF Zephyr tool
                        .setLensIntrinsics(configuredWebcam.cameraCalibration.focalLengthX,
                                configuredWebcam.cameraCalibration.focalLengthY,
                                configuredWebcam.cameraCalibration.opticalCenterX,
                                configuredWebcam.cameraCalibration.opticalCenterY)
                        .build();

                activeProcessor = aprilTagProcessor;
                webcamFrameProcessor = null;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized VisionPortal processor");
        }

        RobotLogCommon.d(TAG, "Opening the webcam " + configuredWebcam.webcamId +
                " with the processor " + configuredWebcam.processorId);
        visionPortal = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                // If set "false", monitor shows camera view without annotations.
                .setAutoStopLiveView(false)

                // Set and enable the processor.
                .addProcessor(activeProcessor)
                //**TODO later - support multiple processors and use .addProcessors(processors)
                // which must be an array VisionProcessor[]

                .build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.webcamId + " on " + configuredWebcam.getWebcamName().getDeviceName());

        // Wait here with timeout until VisionPortal.CameraState.STREAMING.
        // The async camera startup happens behind the scenes in VisionPortalImpl.
        // This is not ideal; see the comments in: WebcamFrameProcessorImpl.init().
        RobotLogCommon.d(TAG, "Waiting for the webcam to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < 2000 && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(50);
        }

        RobotLogCommon.d(TAG, "The webcam is streaming");
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            throw new AutonomousRobotException(TAG, "Timed out waiting for webcam streaming to start");

        // Start with the processor disabled.
        visionPortal.setProcessorEnabled(activeProcessor, false);
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
    }

    public void enableWebcamFrameProcessor() {
       if (activeProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME)
           return; // already enabled

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS)
          visionPortal.setProcessorEnabled(activeProcessor, false);

        if (webcamFrameProcessor == null)
            throw new AutonomousRobotException(TAG, "Attempt to enable an uninitialized WEBCAM_FRAME processor");

        visionPortal.setProcessorEnabled(webcamFrameProcessor, true);
        activeProcessor = webcamFrameProcessor;
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME;
        RobotLogCommon.d(TAG, "Enabling the processor " + RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);
    }

    public void enableAprilTagProcessor() {
        if (activeProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)
            return; // already enabled

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS)
            visionPortal.setProcessorEnabled(activeProcessor, false);

        if (aprilTagProcessor == null)
            throw new AutonomousRobotException(TAG, "Attempt to enable an uninitialized APRIL_TAG processor");

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        activeProcessor = aprilTagProcessor;
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG;
        RobotLogCommon.d(TAG, "Enabling the processor " + RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG);
    }

    public void disableWebcamFrameProcessor() {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME)
            return; // already disabled

        if (webcamFrameProcessor == null)
            throw new AutonomousRobotException(TAG, "Attempt to disable an uninitialized WEBCAM_FRAME processor");

        visionPortal.setProcessorEnabled(webcamFrameProcessor, false);
        activeProcessor = null;
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
        RobotLogCommon.d(TAG, "Disabling the processor " + RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);
    }

    public void disableAprilTagProcessor() {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)
            return; // already disabled

        if (aprilTagProcessor == null)
            throw new AutonomousRobotException(TAG, "Attempt to disable an uninitialized APRIL_TAG processor");

        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        activeProcessor = null;
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
        RobotLogCommon.d(TAG, "Enabling the processor " + RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG);
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
        RobotLogCommon.d(TAG, "Stop streaming the webcam " + configuredWebcam.webcamId);
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
        RobotLogCommon.d(TAG, "Resume streaming the webcam " + configuredWebcam.webcamId);
    }

    // Completely shut down the webcam.
    // To be called from the finally block of FTCAuto or any TeleOp
    // OpMode that uses the webcam.
    public void finalShutdown() {
        visionPortal.close();
        RobotLogCommon.d(TAG, "Final shutdown of the webcam " + configuredWebcam.webcamId);
    }

    public Pair<Mat, Date> getVisionPortalWebcamData(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME)
            throw new AutonomousRobotException(TAG, "WEBCAM_FRAME is not the active processor");

        Pair<Mat, Date> frameVal = null;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            frameVal = webcamFrameProcessor.getWebcamFrame();
            if (frameVal != null)
                break;
            else {
                RobotLogCommon.v(TAG, "No available webcam frame");
                sleep(50);
            }
        }

        return frameVal;
    }

    public List<AprilTagDetection> getAprilTagData(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)
            throw new AutonomousRobotException(TAG, "APRIL_TAG is not the active processor");

        List<AprilTagDetection> currentDetections = new ArrayList<>();
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            currentDetections = aprilTagProcessor.getDetections();
            if (!currentDetections.isEmpty())
                break;
            else {
                RobotLogCommon.v(TAG, "No available AprilTag");
                sleep(50);
            }
        }

        return currentDetections;
    }
}
