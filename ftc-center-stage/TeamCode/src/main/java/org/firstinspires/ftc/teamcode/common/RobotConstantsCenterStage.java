package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsCenterStage {

    public enum OpMode {
        // Autonomous OpModes
        BLUE_A2, BLUE_A4, RED_F2, RED_F4,
        TEST, TEST_PRE_MATCH, AUTO_NO_DRIVE,

        // TeleOp OpModes
        TELEOP_NO_DRIVE,

        // Pseudo OpModes for running Autonomous actions from within
        // TeleOp. These are not "real" OpMoces in that they don't
        // appear on the Driver Station but they are present in
        // RobotAction.xml.
        TELEOP_TAKE_PICTURE_WEBCAM,

        // Pseudo OpModes for running EasyOpenCV webcam calibration
        // from TeleOp. These are also not "real" OpMoces in that
        // they don't appear on the Driver Station but they are
        // present in RobotAction.xml.
        TEAM_PROP_CALIBRATION
    }

    // The CameraId identifies each unique camera and its position on
    // the robot.
    public enum InternalWebcamId {
        FRONT_WEBCAM, REAR_WEBCAM,
        WEBCAM_NPOS
    }

    public enum ProcessorIdentifier {
        WEBCAM_FRAME, APRIL_TAG, PROCESSOR_NPOS
    }

    public enum TeamPropRecognitionPath {
        COLOR, CIRCLES, COLOR_CHANNEL_GRAYSCALE
    }

    public enum TeamPropLocation {
        LEFT_SPIKE, CENTER_SPIKE, RIGHT_SPIKE
    }

    // AprilTag identifiers
    public enum AprilTagIdentifier {
        RED_ALLIANCE_AUDIENCE_WALL, RED_ALLIANCE_PIXEL_STACK,
        BLUE_ALLIANCE_AUDIENCE_WALL, BLUE_ALLIANCE_PIXEL_STACK,
        RED_ALLIANCE_BACKDROP_LEFT, RED_ALLIANCE_BACKDROP_CENTER, RED_ALLIANCE_BACKDROP_RIGHT,
        BLUE_ALLIANCE_BACKDROP_LEFT, BLUE_ALLIANCE_BACKDROP_CENTER, BLUE_ALLIANCE_BACKDROP_RIGHT
    }

}