package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

// Support a single camera and one or more processors, e.g. a RawFrameProcessor
// and an AprilTag processor. Even if multiple processors are identified in the
// webcam configuration, the XML element <START_WEBCAM> in RobotAction.xml
// controls which processors are attached to the webcam for a particular
// task or tasks and which one, if any, of the processors is enabled when the
// webcam is started. For TeleOp Opmodes such as the SpikeWindowViewer the
// name(s) of the processor(s) must be supplied when the webcam is started.

//&& To support both a RawFrameProcessor and an AprilTagProcessor on the
// same webcam you need a DualProcessorWebcam that implements both
// RawFrameProvider and AprilTagProvider.

// It is an application-wide assumption that there is only one instance of
// this class per webcam and that its methods are called from a single thread.
public class VisionPortalWebcam {
    private static final String TAG = VisionPortalWebcam.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    private final EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors;
    protected RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
    protected VisionProcessor activeProcessor;
    protected final VisionPortal visionPortal;

    // Constructor that can be used to attach a single processor to a webcam.
    public VisionPortalWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                              RobotConstantsCenterStage.ProcessorIdentifier pProcessorId,
                              Pair<VisionProcessor, Boolean> pAssignedProcessor) {
        this(pConfiguredWebcam,
                new EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>>(RobotConstantsCenterStage.ProcessorIdentifier.class)
                {{put(pProcessorId, pAssignedProcessor);}});
    }

    public VisionPortalWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                              EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> pAssignedProcessors) {
        configuredWebcam = pConfiguredWebcam;
        assignedProcessors = pAssignedProcessors;
        RobotLogCommon.d(TAG, "Preparing to open the webcam " + configuredWebcam.internalWebcamId);

        // Check that the processors assigned to this webcam belong to the camera's <processor_set>
        // in RobotConfig.xml.
        ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processorIdentifiers = configuredWebcam.processorIdentifiers;
        for (Map.Entry<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> entry : assignedProcessors.entrySet()) {
            if (!processorIdentifiers.contains(entry.getKey()))
                throw new AutonomousRobotException(TAG, "Assigned processor with id " + entry.getValue().first + " is not in the configuration for webcam " + configuredWebcam.internalWebcamId);
        }

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false);

        // Add all assigned processors here, which will enable them.
        assignedProcessors.forEach((k, v) -> builder.addProcessor(v.first));
        visionPortal = builder.build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.internalWebcamId + " on " + pConfiguredWebcam.getWebcamName().getDeviceName());

        // Check that only one webcam may be marked for "enable_on_start";
        // disable all others. It is also possible that all processors may be disabled
        // on start.
        AtomicInteger enabledCount = new AtomicInteger();
        assignedProcessors.forEach((k, v) -> {
            if (v.second) {
                if (enabledCount.addAndGet(1) > 1)
                    throw new AutonomousRobotException(TAG, "Only 1 processor may be enabled on start");
                activeProcessorId = k;
                activeProcessor = v.first;
                RobotLogCommon.d(TAG, "Processor enabled on start " + activeProcessorId);
            }
            else {
                visionPortal.setProcessorEnabled(v.first, false);
                RobotLogCommon.d(TAG, "Processor disabled on start " + k);
            }
        });

        // The async camera startup happens behind the scenes in VisionPortalImpl.
    }

    // Wait here with timeout until VisionPortal.CameraState.STREAMING but
    //**TODO Why? STARTING_STREAM also seems to be good enough.
    public boolean waitForWebcamStart(int pTimeoutMs) {
        RobotLogCommon.d(TAG, "Waiting for webcam " + configuredWebcam.internalWebcamId + " to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < pTimeoutMs &&
                !(visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM ||
                        visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)) {
            sleep(50);
        }

        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        RobotLogCommon.d(TAG, "State of webcam " + configuredWebcam.internalWebcamId + ": " + cameraState);
        if (!(cameraState == VisionPortal.CameraState.STARTING_STREAM || cameraState == VisionPortal.CameraState.STREAMING)) {
            RobotLogCommon.d(TAG, "Timed out waiting for webcam streaming to start");
            return false;
        }

        return true;
    }

    public Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> getActiveProcessor() {
        return Pair.create(activeProcessorId, activeProcessor);
    }

    // Adapted from the FTC SDK 9.0 sample RobotAutoDriveToAprilTagOmni.
    /*
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag().
     */
    public void setManualExposure(int exposureMS, int gain, int pTimeoutMs) {
        // Wait for the camera to be open, then use the controls
        // Make sure camera is streaming before we try to set the exposure controls
        if (!(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM)) {
            boolean webcamIsStreaming = false;
            ElapsedTime streamingTimer = new ElapsedTime();
            streamingTimer.reset(); // start
            while (!(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                    visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM)
                    && streamingTimer.milliseconds() < pTimeoutMs) {
                sleep(20);
            }

            if (!webcamIsStreaming)
                throw new AutonomousRobotException(TAG, "Timed out waiting for CameraState.STREAMING");
        }

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }

        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }

    public void enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (!(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM))
            throw new AutonomousRobotException(TAG, "Attempting to enable the processor " + pProcessorId + " even though the webcam is not STREAMING");

        VisionProcessor processor = assignedProcessors.get(pProcessorId).first;
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to enable an unassigned processor " + pProcessorId);

        if (pProcessorId == activeProcessorId) {
            RobotLogCommon.d(TAG, "Ignoring request to enable processor " + pProcessorId + " which is already enabled");
            return;
        }

        // In our application we only allow one processor at a time to be enabled.
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS) {
            RobotLogCommon.d(TAG, "The processor " + activeProcessorId + " is active, disable it");
            visionPortal.setProcessorEnabled(activeProcessor, false);
        }

        visionPortal.setProcessorEnabled(processor, true);
        activeProcessorId = pProcessorId;
        activeProcessor = processor;
        RobotLogCommon.d(TAG, "Enabling the processor " + pProcessorId);
    }

    public void disableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (!(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM))
            throw new AutonomousRobotException(TAG, "Attempting to disable the processor " + pProcessorId + " even though the webcam is not STREAMING");

        VisionProcessor processor = assignedProcessors.get(pProcessorId).first;
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to disable an unassigned processor " + pProcessorId);

        if (activeProcessorId != pProcessorId) {
            RobotLogCommon.d(TAG, "Request to disable the processor " + pProcessorId + " which is not active");
            RobotLogCommon.d(TAG, "The active processor is " + activeProcessorId);
            return;
        }

        visionPortal.setProcessorEnabled(processor, false);
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
        activeProcessor = null;
        RobotLogCommon.d(TAG, "Disabling the processor " + pProcessorId);
    }

    public void stopStreaming() {
        if (!(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM)) {
            RobotLogCommon.d(TAG, "Ignoring request to stop streaming the webcam " + configuredWebcam.internalWebcamId);
            RobotLogCommon.d(TAG, "The webcam is not streaming");
        } else
            visionPortal.stopStreaming();
    }

    public boolean resumeStreaming(int pTimeoutMs) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM) {
            RobotLogCommon.d(TAG, "Ignoring request to resume streaming the webcam " + configuredWebcam.internalWebcamId);
            RobotLogCommon.d(TAG, "The webcam is already streaming");
            return true;
        }

        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        visionPortal.resumeStreaming();
        while (streamingTimer.milliseconds() < pTimeoutMs && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(50);
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM) {
            RobotLogCommon.d(TAG, "Resumed streaming the webcam " + configuredWebcam.internalWebcamId);
            return true;
        }

        RobotLogCommon.d(TAG, "Timed out " + pTimeoutMs + " while attempting to resume streaming the webcam " + configuredWebcam.internalWebcamId);
        return false;
    }

    // Completely shut down the webcam.
    // To be called from the finally block of FTCAuto or any TeleOp
    // OpMode that uses the webcam.
    public void finalShutdown() {
        //&& avoid crash in easyopencv by simply closing the camera ...
        // Reported to FTC
        /*
        // Shut down the active processor. Stop streaming.
        if (activeProcessorEnabled && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            disableProcessor();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
          visionPortal.stopStreaming();
         */

        visionPortal.close();
        RobotLogCommon.d(TAG, "Final shutdown of the webcam " + configuredWebcam.internalWebcamId);
    }

}
