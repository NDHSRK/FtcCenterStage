
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

import java.util.concurrent.TimeUnit;

// Use the VisionPortal API to manage a webcam and either one or two
// "processors". The processors are constructed in advance and
// passed in to this class.
public abstract class VisionPortalWebcam {
    private static final String TAG = VisionPortalWebcam.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    private final VisionPortal visionPortal;
    protected RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId;
    protected final VisionProcessor activeProcessor;
    private boolean activeProcessorEnabled;

    // Support a single camera and a single processor, a WebcamFrameProcessor
    // or an AprilTag processor.
    public VisionPortalWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                              Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pAssignedProcessor) {
        configuredWebcam = pConfiguredWebcam;
        activeProcessorId = pAssignedProcessor.first;
        activeProcessor = pAssignedProcessor.second;
        RobotLogCommon.d(TAG, "Preparing to open the webcam " + configuredWebcam.internalWebcamId);

        visionPortal = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(pAssignedProcessor.second)
                //## 11/06/23 need to enable LiveView for testing with scrcpy
                //.enableLiveView(true) //**TODO set true 11/21/23
                .enableLiveView(false) //##PY changed to false 10/5/23 - and must remain false
                //.setAutoStopLiveView(false) //## Only if we're using LiveView

                .build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.internalWebcamId + " on " + pConfiguredWebcam.getWebcamName().getDeviceName());

        // The async camera startup happens behind the scenes in VisionPortalImpl.
    }

    // Wait here with timeout until VisionPortal.CameraState.STREAMING.
    public boolean waitForWebcamStart(int pTimeoutMs) {
        RobotLogCommon.d(TAG, "Waiting for webcam " + configuredWebcam.internalWebcamId + " to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < pTimeoutMs && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(50);
        }

        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        RobotLogCommon.d(TAG, "State of webcam " + configuredWebcam.internalWebcamId + ": " + cameraState);
        if (cameraState != VisionPortal.CameraState.STREAMING) {
            RobotLogCommon.d(TAG, "Timed out waiting for webcam streaming to start");
            return false;
        }

        activeProcessorEnabled = true;
        return true;
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

    public void enableProcessor() {
       if (activeProcessorEnabled) {
            RobotLogCommon.d(TAG, "Ignoring request to enable processor " + activeProcessorId + " which is already active");
            return; // already enabled
        }

        visionPortal.setProcessorEnabled(activeProcessor, true);
        activeProcessorEnabled = true;
        RobotLogCommon.d(TAG, "Enabling the processor " + activeProcessorId);
    }

    public void disableProcessor() {
         if (!activeProcessorEnabled) {
            RobotLogCommon.d(TAG, "Request to disable the processor " + activeProcessorId + " which is already disabled");
            return; // already disabled
        }

        visionPortal.setProcessorEnabled(activeProcessor, false);
        activeProcessorEnabled = false;
        RobotLogCommon.d(TAG, "Disabling the processor " + activeProcessorId);
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
        if (visionPortal == null)
            return; // already shut down

        // Shut down the active processor. Stop streaming.
        //**TODO crash in disableProcessor()? Loop until processor disabled?
        if (activeProcessorEnabled && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            disableProcessor();
        
        //**TODO crash in stopStreaming()? Loop until !STREAMING?
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
          visionPortal.stopStreaming();

        visionPortal.close();
        RobotLogCommon.d(TAG, "Final shutdown of the webcam " + configuredWebcam.internalWebcamId);
    }

}
