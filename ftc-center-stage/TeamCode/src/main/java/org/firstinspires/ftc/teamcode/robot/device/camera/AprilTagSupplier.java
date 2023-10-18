package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public interface AprilTagSupplier {

    public List<AprilTagDetection> getAprilTagData(int pTimeoutMs);
}
