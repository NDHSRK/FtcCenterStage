package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.opencv.core.Mat;

import java.util.Date;

public interface ImageProvider {

    // Images must always be returned in the BGR format.
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    Pair<Mat, Date> getImage() throws InterruptedException;
}
