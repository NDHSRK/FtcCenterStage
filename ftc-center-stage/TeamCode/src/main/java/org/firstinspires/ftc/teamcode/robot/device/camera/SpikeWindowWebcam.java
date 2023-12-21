package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.EnumMap;

public class SpikeWindowWebcam extends VisionPortalWebcam {
    private static final String TAG = SpikeWindowWebcam.class.getSimpleName();

    public SpikeWindowWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                             EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> pAssignedProcessors) {
        super(pConfiguredWebcam, pAssignedProcessors);

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.SPIKE_WINDOW)
            throw new AutonomousRobotException(TAG, "SPIKE_WINDOW is not the active processor");
    }

}
