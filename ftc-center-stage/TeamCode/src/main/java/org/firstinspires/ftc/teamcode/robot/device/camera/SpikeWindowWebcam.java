package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.Date;
import java.util.Objects;

public class SpikeWindowWebcam extends VisionPortalWebcam {
    private static final String TAG = SpikeWindowWebcam.class.getSimpleName();

    public SpikeWindowWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                             Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pAssignedProcessor) {
        super(pConfiguredWebcam, pAssignedProcessor);

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.SPIKE_WINDOW)
            throw new AutonomousRobotException(TAG, "SPIKE_WINDOW is not the active processor");
    }

}
