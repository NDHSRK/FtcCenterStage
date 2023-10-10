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
        COLOR_CHANNEL_CIRCLES, COLOR_CHANNEL_FEATURES, COLOR_CHANNEL_CONTOURS,
        COLOR_CHANNEL_BRIGHT_SPOT, GRAYSCALE_BRIGHT_SPOT
    }

    // Relative position of a barcode element within the ROI.
    public enum SpikeLocationWindow {
        LEFT, RIGHT, WINDOW_NPOS
    }

    // Constructor parameters are the AprilTag id of the
    // blue alliance backstop locations and the AprilTag
    // id of the red alliance backstop locations.
    public enum TeamPropLocation {
        LEFT_SPIKE(1, 4), CENTER_SPIKE (2, 5), RIGHT_SPIKE(3, 6), SPIKE_NPOS(-1, -1);

        private final int blueBackdropAprilTagId;
        private final int redBackdropAprilTagId;
        TeamPropLocation(int pBlueBackdropAprilTagId, int pRedBackdropAprilTagId) {
            blueBackdropAprilTagId = pBlueBackdropAprilTagId;
            redBackdropAprilTagId = pRedBackdropAprilTagId;
        }

        public int getBlueBackdropAprilTagId() {
            return blueBackdropAprilTagId;
        }

        public int getRedBackdropAprilTagId() {
            return redBackdropAprilTagId;
        }
    }

    // AprilTag identifiers
    public enum FieldWallAprilTagIdentifier {
        RED_ALLIANCE_AUDIENCE_WALL, RED_ALLIANCE_PIXEL_STACK,
        BLUE_ALLIANCE_AUDIENCE_WALL, BLUE_ALLIANCE_PIXEL_STACK
    }

    // Reference implementation of the gold cube.
    public enum GoldCubeRecognitionPath {
        RED_CHANNEL_GRAYSCALE, COLOR
    }

}