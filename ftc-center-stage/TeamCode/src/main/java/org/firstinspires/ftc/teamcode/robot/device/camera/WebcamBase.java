
package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.EnumMap;
import java.util.concurrent.TimeUnit;

// Use the VisionPortal API to manage a webcam and either one or two
// "processors". The processors are constructed in advance and
// passed in to this class.
public abstract class WebcamBase {
    private static final String TAG = WebcamBase.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    protected final EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> processors;
    private int processorIndex = 0;

    protected RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId;
    private final VisionPortal visionPortal;

    // Support a single camera and oe or two processors, a WebcamFrameProcessor,
    // an AprilTag processor, or both. Even if both processors are in the
    // configuration, only one at a time can be enabled. See the sample ConceptDoubleVision.
    public WebcamBase(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam) {
        configuredWebcam = pConfiguredWebcam;
        RobotLogCommon.d(TAG, "Preparing to open the webcam " + configuredWebcam.internalWebcamId);

        processors = configuredWebcam.getProcessors();
        VisionProcessor[] processorArray = new VisionProcessor[processors.size()];
        processors.forEach((processorId, processor) -> {
            RobotLogCommon.d(TAG, "Adding processor " + processorId);
            processorArray[processorIndex++] = processor;
        });

        visionPortal = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false) //##PY changed to false 10/5/23 - and must remain false
                // If set "false", monitor shows camera view without annotations.
                .setAutoStopLiveView(false) //**TODO we're not using LiveView

                // Set and enable the processor(s).
                .addProcessors(processorArray)

                .build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.internalWebcamId + " on " + pConfiguredWebcam.getWebcamName().getDeviceName());

        // Wait here with timeout until VisionPortal.CameraState.STREAMING.
        // The async camera startup happens behind the scenes in VisionPortalImpl.
        RobotLogCommon.d(TAG, "Waiting for webcam " + configuredWebcam.internalWebcamId + " to start streaming");
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
        processors.forEach((processorId, processor) ->
                visionPortal.setProcessorEnabled(processor, false));
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
    }

    public RobotConstantsCenterStage.InternalWebcamId getInternalWebcamId() {
        return configuredWebcam.internalWebcamId;
    }

    // Adapted from the FTC SDK 9.0 sample RobotAutoDriveToAprilTagOmni.
    /*
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag().
     */
    public void setManualExposure(int exposureMS, int gain, int pTimeoutMs) {
        // Wait for the camera to be open, then use the controls
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            boolean webcamIsStreaming = false;
            ElapsedTime streamingTimer = new ElapsedTime();
            streamingTimer.reset(); // start
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && streamingTimer.milliseconds() < pTimeoutMs) {
                sleep(20);
            }

            //**TODO leave in for testing but crashing is too dire for competition -
            // just log, give up and return.
            if (!webcamIsStreaming)
                throw new AutonomousRobotException(TAG, "Timed out waiting for CameraState.STREAMING");
        }

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }

        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }

    public void enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (pProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS) {
            RobotLogCommon.d(TAG, "Ignoring request to enable processor " + RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS);
            return;
        }

        if (activeProcessorId == pProcessorId) {
            RobotLogCommon.d(TAG, "Ignoring request to enable processor " + pProcessorId + " which is already active");
            return; // already enabled
        }

        VisionProcessor processor = processors.get(pProcessorId);
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to enable an uninitialized processor " + pProcessorId);

        // In our application we only allow one processor at a time to be enabled.
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS) {
            RobotLogCommon.d(TAG, "The processor " + activeProcessorId + " is already active, disable it");
            VisionProcessor activeProcessor = processors.get(activeProcessorId);
            visionPortal.setProcessorEnabled(activeProcessor, false);
        }

        visionPortal.setProcessorEnabled(processor, true);
        activeProcessorId = pProcessorId;
        RobotLogCommon.d(TAG, "Enabling the processor " + pProcessorId);
    }

    public void disableProcessor(RobotConstantsCenterStage.ProcessorIdentifier pProcessorId) {
        if (pProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS) {
            RobotLogCommon.d(TAG, "Ignoring request to disablethe  processor " + RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS);
            return;
        }

        if (activeProcessorId != pProcessorId) {
            RobotLogCommon.d(TAG, "Request to disable the processor " + pProcessorId + " which is not active");
            RobotLogCommon.d(TAG, "The active processor is " + activeProcessorId);
            return; // already disabled
        }

        VisionProcessor processor = processors.get(pProcessorId);
        if (processor == null)
            throw new AutonomousRobotException(TAG, "Attempt to disable an uninitialized processor " + pProcessorId);

        visionPortal.setProcessorEnabled(processor, false);
        activeProcessorId = RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS;
        RobotLogCommon.d(TAG, "Disabling the processor " + pProcessorId);
    }

    public void stopStreaming() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            RobotLogCommon.d(TAG, "Ignoring request to stop streaming the webcam " + configuredWebcam.internalWebcamId);
            RobotLogCommon.d(TAG, "The webcam is not streaming");
            return;
        }

        visionPortal.stopStreaming();
        RobotLogCommon.d(TAG, "Stop streaming the webcam " + configuredWebcam.internalWebcamId);
    }

    public void resumeStreaming() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            RobotLogCommon.d(TAG, "Ignoring request to resume streaming the webcam " + configuredWebcam.internalWebcamId);
            RobotLogCommon.d(TAG, "The webcam is already streaming");
            return;
        }

        visionPortal.resumeStreaming();
        RobotLogCommon.d(TAG, "Resume streaming the webcam " + configuredWebcam.internalWebcamId);
    }

    // Completely shut down the webcam.
    // To be called from the finally block of FTCAuto or any TeleOp
    // OpMode that uses the webcam.
    public void finalShutdown() {
        // Shut down the active processor, if any. Stop streaming.
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.PROCESSOR_NPOS)
            disableProcessor(activeProcessorId);

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            visionPortal.stopStreaming();

        visionPortal.close();
        RobotLogCommon.d(TAG, "Final shutdown of the webcam " + configuredWebcam.internalWebcamId);
    }

}
