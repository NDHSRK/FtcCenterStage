package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {

    public static final String IMAGE_DIR = "/images/";
    public static final String LOG_DIR = "/logs/";
    public static final String XML_DIR = "/xml/";

    // For standard indented formatting of an XML file.
    public static final String xsltFilename = "StandardTransform.xslt";

    public enum RunType {
        AUTONOMOUS, AUTO_NO_DRIVE,
        TELEOP, TELEOP_WITH_IMU,
        TELEOP_WITH_EMBEDDED_AUTONOMOUS,
        TELEOP_NO_DRIVE, TELEOP_OPENCV_CALIBRATION,
        TELEOP_NO_DRIVE_WITH_EMBEDDED_AUTONOMOUS
    }

    public enum Alliance {
        BLUE, RED, NONE
    }

    public enum RecognitionResults {
        RECOGNITION_INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

}
