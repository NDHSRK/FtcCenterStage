package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class AprilTagWebcam extends VisionPortalWebcam implements AprilTagSupplier {
    private static final String TAG = AprilTagWebcam.class.getSimpleName();

    public AprilTagWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                          Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pAssignedProcessor) {
        super(pConfiguredWebcam, pAssignedProcessor);
    }

    public List<AprilTagDetection> getAprilTagData(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)
            throw new AutonomousRobotException(TAG, "APRIL_TAG is not the active processor");

        AprilTagProcessor aprilTagProcessor = (AprilTagProcessor) activeProcessor;

        //**TODO this code is common with FrontWebcamContainer - it can be static somewhere.
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
