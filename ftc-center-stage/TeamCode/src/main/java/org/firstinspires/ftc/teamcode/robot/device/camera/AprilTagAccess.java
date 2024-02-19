package org.firstinspires.ftc.teamcode.robot.device.camera;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class AprilTagAccess {
    private static final String TAG = AprilTagAccess.class.getSimpleName();

    // Returns an empty List if no AprilTag detections are available.
    public static List<AprilTagDetection> getAprilTagData(AprilTagProcessor pAprilTagProcessor, int pTimeoutMs) {
        List<AprilTagDetection> currentDetections = null;
        ElapsedTime dataAcquiredTimer = new ElapsedTime();
        dataAcquiredTimer.reset(); // start
        while (dataAcquiredTimer.milliseconds() < pTimeoutMs) {
            // The FTC samples use getDetections but we always want the
            // //latest - so use getFreshDetections()
            currentDetections = Objects.requireNonNull(pAprilTagProcessor,
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
