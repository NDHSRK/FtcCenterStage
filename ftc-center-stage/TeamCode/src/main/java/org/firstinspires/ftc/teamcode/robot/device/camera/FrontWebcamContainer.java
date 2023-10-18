package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Objects;

public class FrontWebcamContainer extends WebcamBase implements WebcamFrameSupplier, AprilTagSupplier {
    private static final String TAG = FrontWebcamContainer.class.getSimpleName();

    public FrontWebcamContainer(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam) {
        super(pConfiguredWebcam);
    }

    public Pair<Mat, Date> getWebcamFrame(int pTimeoutMs) {
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
