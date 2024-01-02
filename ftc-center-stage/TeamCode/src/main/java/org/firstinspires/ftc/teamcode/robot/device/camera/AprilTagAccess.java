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

public class AprilTagAccess implements AprilTagProvider {
    private static final String TAG = AprilTagAccess.class.getSimpleName();

    private final AprilTagProcessor aprilTagProcessor;

    public AprilTagAccess(AprilTagProcessor pAprilTagProcessor) {
        aprilTagProcessor = pAprilTagProcessor;
    }

    // Returns an empty List if no AprilTag detections are available.
    public List<AprilTagDetection> getAprilTagData(int pTimeoutMs) {
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
