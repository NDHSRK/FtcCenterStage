package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.EnumMap;

//**TODO Don't need to inherit from VisionPortalWebcam
public class CameraStreamWebcam extends VisionPortalWebcam {
    private static final String TAG = CameraStreamWebcam.class.getSimpleName();

    public CameraStreamWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                              RobotConstantsCenterStage.ProcessorIdentifier pProcessorId,
                              Pair<VisionProcessor, Boolean> pAssignedProcessor) {
        super(pConfiguredWebcam, pProcessorId, pAssignedProcessor);

        if (!(activeProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.SPIKE_WINDOW ||
                activeProcessorId == RobotConstantsCenterStage.ProcessorIdentifier.PIXEL_COUNT))
            throw new AutonomousRobotException(TAG, "Active processor " + activeProcessorId + " not compatible with CameraStreamWebcam");
    }

}
