package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.opencv.core.Mat;

import java.util.Date;

// This class provides compatibility with the IntelliJ project IJCenterStage,
// which reads images from a file.
public class WebcamImage implements ImageProvider {

    private static final String TAG = WebcamImage.class.getSimpleName();
    
    private final WebcamFrameWebcam webcamFrameWebcam;

    public WebcamImage(WebcamFrameWebcam pWebcamFrameWebcam) {
        webcamFrameWebcam = pWebcamFrameWebcam;
    }

    //!! Warning - may return null if the timer expires without a frame.
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    @Override
    public Pair<Mat, Date> getImage() {
        Pair<Mat, Date> cameraFramData = webcamFrameWebcam.getWebcamFrame(1000);
        if (cameraFramData == null)
            RobotLogCommon.d(TAG, "Timed out waiting for a video frame from the vision Portal webcam");

        return cameraFramData;
    }

}
