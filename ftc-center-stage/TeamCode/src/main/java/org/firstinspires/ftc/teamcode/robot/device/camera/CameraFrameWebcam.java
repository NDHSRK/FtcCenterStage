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

public class CameraFrameWebcam extends VisionPortalWebcam implements WebcamFrameSupplier {
    private static final String TAG = CameraFrameWebcam.class.getSimpleName();

    public CameraFrameWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                             Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pAssignedProcessor) {
        super(pConfiguredWebcam, pAssignedProcessor);
    }

    public Pair<Mat, Date> getWebcamFrame(int pTimeoutMs) {
        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME)
            throw new AutonomousRobotException(TAG, "WEBCAM_FRAME is not the active processor");

        WebcamFrameProcessor webcamFrameProcessor = (WebcamFrameProcessor) activeProcessor;

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

}
