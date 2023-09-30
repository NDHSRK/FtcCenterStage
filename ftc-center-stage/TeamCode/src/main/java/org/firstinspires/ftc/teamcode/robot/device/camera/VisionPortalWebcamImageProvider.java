package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.opencv.core.Mat;

import java.util.Date;

// This class provides compatibility with the IntelliJ project IJCenterStage,
// which reads images from a file.
public class VisionPortalWebcamImageProvider implements ImageProvider {

    private static final String TAG = VisionPortalWebcamImageProvider.class.getSimpleName();
    
    private final VisionPortalWebcam visionPortalWebcam;

    public VisionPortalWebcamImageProvider(VisionPortalWebcam pVisionPortalWebcam) {
        visionPortalWebcam = pVisionPortalWebcam;
    }

    //!! Warning - may return null if the timer expires without a frame.
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    @Override
    public Pair<Mat, Date> getImage() {
        Pair<Mat, Date> visionPortalData = visionPortalWebcam.getVisionPortalWebcamData(1000);
        if (visionPortalData == null)
            RobotLogCommon.d(TAG, "Timed out waiting for a video frame from the vision Portal webcam");

        return visionPortalData;
    }

}
