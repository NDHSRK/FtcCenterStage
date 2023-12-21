package org.firstinspires.ftc.teamcode.robot.device.camera;
import static android.os.SystemClock.sleep;
import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

// Support a single camera and one or more processors, e.g. a RawFrameProcessor
// and an AprilTag processor. Even if multiple processors are identified in the
// webcam configuration, the XML element <START_WEBCAM> in RobotAction.xml
// controls which processors are attached to the webcam for a particular
// task or tasks and which one, if any, of the processors is enabled when the
// webcam is started. For TeleOp Opmodes such as the SpikeWindowViewer the
// name(s) of the processor(s) must be supplied when the webcam is started.

//**TODO To support both a RawFrameProcessor and an AprilTagProcessor on the
// same webcam you need a DualProcessorWebcam that implements both
// RawFrameProvider and AprilTagProvider.

//**TODO You also need a way to parse out multiple <processor> elements
// under <START_WEBCAM> - see getStartWebcamProcessors in
// RobotActionXMLCenterStage.java. You need to make sure that these
// processors belong to the camera's <processor_set> in RobotConfig.xml.

//**TODO Check FTCRobot to make sure it allows multiple processors

//**TODO attribute for a <processor> element under <START_WEBCAM>,
// e.g. <processor enable_on_start="raw_frame">. Only one processor
// may be enabled on start - *check this*; if no processors are enabled
// on start then an <ENABLE_PROCESSOR> element must be present in
// RobotAction.xml.

//**TODO support timeouts on stopping and resuming the stream; you need
// an XML element <timeout_ms>

//**TODO Enforce only one processor enabled at a time? So if there's a
// call to enable a processor and a different one is currently enabled,
// automatically disable it? YES

// It is an application-wide assumption that there is only one instance of
// this class per webcam and that its methods are called from a single thread.
public class VisionPortalWebcam2 {
    private static final String TAG = VisionPortalWebcam2.class.getSimpleName();

    private final VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam;
    private final EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors;
    private RobotConstantsCenterStage.ProcessorIdentifier activeProcessorId;

    private final VisionPortal visionPortal;

    public VisionPortalWebcam2(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                               EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> pAssignedProcessors)
    {
        configuredWebcam = pConfiguredWebcam;
        assignedProcessors = pAssignedProcessors;
        RobotLogCommon.d(TAG, "Preparing to open the webcam " + configuredWebcam.internalWebcamId);

        // Check that the processors assigned to this webcam belong to the camera's <processor_set>
        // in RobotConfig.xml.
        ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processorIdentifiers = configuredWebcam.processorIdentifiers;
        for (Map.Entry<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> entry : assignedProcessors.entrySet()) {
            if (!processorIdentifiers.contains(entry.getValue().first))
                throw new AutonomousRobotException(TAG, "Assigned processor with id " + entry.getValue().first + " is not in the configuration for webcam " + configuredWebcam.internalWebcamId);
        }

         VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(configuredWebcam.getWebcamName())
                .setCameraResolution(new Size(configuredWebcam.resolutionWidth, configuredWebcam.resolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false);

        // Add all assigned processors here, which will enable them.
        assignedProcessors.forEach((k,v) -> builder.addProcessor(v.first));
        visionPortal = builder.build();

        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR)
            throw new AutonomousRobotException(TAG, "Error in opening webcam " + configuredWebcam.internalWebcamId + " on " + pConfiguredWebcam.getWebcamName().getDeviceName());

        // It is guaranteed that only one webcam may be marked for "enable_on_start";
        // disable all others.
        assignedProcessors.forEach((k,v) -> {
            if (v.second)
                activeProcessorId = k;
            else
                visionPortal.setProcessorEnabled(v.first, false);
        });

        // The async camera startup happens behind the scenes in VisionPortalImpl.

    }
}
