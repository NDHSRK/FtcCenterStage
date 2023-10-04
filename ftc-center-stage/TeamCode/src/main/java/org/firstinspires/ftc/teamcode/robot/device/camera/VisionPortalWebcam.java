
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;

// Use the VisionPortal API to manage a webcam and either one or two
// "processors": one for raw webcam frames and the other for AprilTags.
//
// It is an application-wide assumption that there is only one instance of
// this class and that its methods are called from a single thread.
public class VisionPortalWebcam {
    private static final String TAG = VisionPortalWebcam.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    private final EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> processors =
            new EnumMap<>(RobotConstantsCenterStage.ProcessorIdentifier.class);
    private int processorIndex = 0;

    private RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId;
    private final VisionPortal visionPortal;

    public VisionPortalWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam) {
        // Support a single camera and two processors, a WebcamFrameProcessor and an
        // AprilTag processor. Even if both processors are in the configuration, only
        // one at a time will be enabled. See the sample ConceptDoubleVision.
        // Note: all processors are attached to the same webcam.
        configuredWebcam = pConfiguredWebcam;
        VisionProcessor[] processorArray = new VisionProcessor[configuredWebcam.processors.size()];
        pConfiguredWebcam.processors.forEach(processorId -> {
            switch (processorId) {
                    case WEBCAM_FRAME: {
                        VisionProcessor webcamFrameProcessor = new WebcamFrameProcessor.Builder().build();
                        processors.put(processorId, webcamFrameProcessor);
                        processorArray[processorIndex++] = webcamFrameProcessor;
                        break;
                    }
                    case APRIL_TAG: {
                        VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
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
                                .setLensIntrinsics(configuredWebcam.cameraCalibration.focalLengthX,
                                        configuredWebcam.cameraCalibration.focalLengthY,
                                        configuredWebcam.cameraCalibration.opticalCenterX,
                                        configuredWebcam.cameraCalibration.opticalCenterY)
                                .build();

                        processors.put(processorId, aprilTagProcessor);
                        processorArray[processorIndex++] = aprilTagProcessor;
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Unrecognized VisionPortal processor");
            }
        });

        RobotLogCommon.d(TAG, "Opening the webcam " + configuredWebcam.internalWebcamId +
                " with the following processor(s): ");
        processors.forEach((processorId,processor) ->
            RobotLogCommon.d(TAG, "Processor " + processorId));

        visionPortal = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                // If set "false", monitor shows camera view without annotations.
                .setAutoStopLiveView(false)

                // Set and enable the processor(s).
                .addProcessors(processorArray)

                .build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.internalWebcamId + " on " + pConfiguredWebcam.getWebcamName().getDeviceName());

        // Wait here with timeout until VisionPortal.CameraState.STREAMING.
        // The async camera startup happens behind the scenes in VisionPortalImpl.
        // This is not ideal; see the comments in: WebcamFrameProcessorImpl.init().
        RobotLogCommon.d(TAG, "Waiting for webcam "  + configuredWebcam.internalWebcamId + " to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < 2000 && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(50);
        }

        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        RobotLogCommon.d(TAG, "State of webcam " + configuredWebcam.internalWebcamId + ": " + cameraState);
        if (cameraState != VisionPortal.CameraState.STREAMING)
            throw new AutonomousRobotException(TAG, "Timed out waiting for webcam streaming to start");

        // Start with the processor(s) disabled.
        processors.forEach((processorId,processor) ->
           visionPortal.setProcessorEnabled(processor, false));
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
    }

    public void enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (activeProcessorId == pProcessorId)
            return; // already enabled

        VisionProcessor processor = processors.get(pProcessorId);
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to enable an uninitialized processor " + pProcessorId);

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS) {
            // A processor is already active, disable it.
            VisionProcessor activeProcessor = processors.get(activeProcessorId);
            visionPortal.setProcessorEnabled(activeProcessor, false);
        }

        visionPortal.setProcessorEnabled(processor, true);
        activeProcessorId = pProcessorId;
        RobotLogCommon.d(TAG, "Enabling the processor " + pProcessorId);
    }

    public void disableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (activeProcessorId != pProcessorId)
            return; // already disabled

        VisionProcessor processor = processors.get(pProcessorId);
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to disable an uninitialized processor " + pProcessorId);

        visionPortal.setProcessorEnabled(processor, false);
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
        RobotLogCommon.d(TAG, "Disabling the processor " + pProcessorId);
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
        RobotLogCommon.d(TAG, "Stop streaming the webcam " + configuredWebcam.internalWebcamId);
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
        RobotLogCommon.d(TAG, "Resume streaming the webcam " + configuredWebcam.internalWebcamId);
    }

    // Completely shut down the webcam.
    // To be called from the finally block of FTCAuto or any TeleOp
    // OpMode that uses the webcam.
    public void finalShutdown() {
        visionPortal.close();
        RobotLogCommon.d(TAG, "Final shutdown of the webcam " + configuredWebcam.internalWebcamId);
    }

    //**TODO Find a way not to hardcode the methods for getting the results from
    // the processors attached to the current webcam.
    public Pair<Mat, Date> getVisionPortalWebcamData(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME)
            throw new AutonomousRobotException(TAG, "WEBCAM_FRAME is not the active processor");

        WebcamFrameProcessor webcamFrameProcessor = (WebcamFrameProcessor) processors.get(RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);

        Pair<Mat, Date> frameVal = null;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            frameVal = Objects.requireNonNull(webcamFrameProcessor).getWebcamFrame();
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

        AprilTagProcessor aprilTagProcessor = (AprilTagProcessor) processors.get(RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG);

        List<AprilTagDetection> currentDetections = new ArrayList<>();
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            currentDetections = Objects.requireNonNull(aprilTagProcessor).getDetections();
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
