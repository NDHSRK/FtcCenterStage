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
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;

public class DualPurposeWebcam extends VisionPortalWebcam implements ImageProvider, AprilTagProvider  {
    private static final String TAG = DualPurposeWebcam.class.getSimpleName();

    public DualPurposeWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                             EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> pAssignedProcessors) {
        super(pConfiguredWebcam, pAssignedProcessors);
    }

    public Pair<Mat, Date> getImage() {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.RAW_FRAME)
            throw new AutonomousRobotException(TAG, "RAW_FRAME is not the active processor");

        RawFrameProcessor rawFrameProcessor = (RawFrameProcessor) activeProcessor;

        Pair<Mat, Date> frameVal = null;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < 1000) {
            frameVal = Objects.requireNonNull(rawFrameProcessor, TAG + " getImage(): rawFrameProcessor unexpectedly null").getWebcamFrame();
            if (frameVal != null)
                break;
            else {
                RobotLogCommon.v(TAG, "No available webcam frame");
                sleep(50);
            }
        }

        return frameVal;
    }

    // Returns an empty List if no AprilTag detections are available.
    public List<AprilTagDetection> getAprilTagData(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)
            throw new AutonomousRobotException(TAG, "APRIL_TAG is not the active processor");

        AprilTagProcessor aprilTagProcessor = (AprilTagProcessor) activeProcessor;
        List<AprilTagDetection> currentDetections = null;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            // The FTC samples use getDetections but we always want the
            // //latest - so use getFreshDetections()
            currentDetections = Objects.requireNonNull(aprilTagProcessor,
                    TAG + " getAprilTagData: aprilTagProcessor is null").getFreshDetections();
            if (currentDetections != null && !currentDetections.isEmpty())
                break;
            else {
                RobotLogCommon.v(TAG, "No available AprilTags");
                sleep(50);
            }
        }

        return currentDetections == null ? new ArrayList<>() : currentDetections;
    }

}
