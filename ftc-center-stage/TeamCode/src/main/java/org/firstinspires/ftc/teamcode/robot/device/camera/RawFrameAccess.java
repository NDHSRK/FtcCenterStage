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

// Use the ImageProvider interface to provide cross-project compatibility
// with IntelliJ feeder projects that get images from files.
public class RawFrameAccess implements ImageProvider {
    private static final String TAG = RawFrameAccess.class.getSimpleName();

    private final RawFrameProcessor rawFrameProcessor;

    public RawFrameAccess(RawFrameProcessor pRawFrameProcessor) {
        rawFrameProcessor = pRawFrameProcessor;
    }

    public Pair<Mat, Date> getImage() {
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

}
