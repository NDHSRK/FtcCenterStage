package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.imu.BasicIMU;
import org.firstinspires.ftc.teamcode.robot.device.imu.IMUReader;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrain;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.EnumMap;
import java.util.Optional;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class FTCRobot {

    private static final String TAG = FTCRobot.class.getSimpleName();

    // All motors on the robot for this year's game.
    public enum MotorId {
        LEFT_FRONT_DRIVE, RIGHT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_BACK_DRIVE,
        ELEVATOR_LEFT, ELEVATOR_RIGHT,
        MOTOR_ID_NPOS // for error checking
    }

    private final HardwareMap hardwareMap;

    public final TeleOpSettings teleOpSettings;
    public final DriveTrain driveTrain;

    public final EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams;

    public final IMUReader imuReader;

    public FTCRobot(LinearOpMode pLinearOpMode, RobotConstants.RunType pRunType)  throws InterruptedException {
        hardwareMap = pLinearOpMode.hardwareMap;

        RobotLogCommon.c(TAG, "FTCRobot constructor");

        //!! WARNING
        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors. [See DriveTrainCore.java]
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.

        //!! 11/16/21 commented out AND NEEDS TO REMAIN OUT because of unpredictable
        // behavior with RUN_TO_POSITION, e.g. immediate reporting of completion of RTP
        // with ending encoder counts of 0 - even though the movement actually took
        // place.
        /*
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        */

        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Get the hardware configuration parameters from RobotConfig.xml.
        try {
            RobotConfigXML configXML = new RobotConfigXML(xmlDirectory);
            XPathAccess configXPath;

            // If we're running a TeleOp OpMode, get the TeleOp settings.
            if (pRunType == RobotConstants.RunType.TELEOP || pRunType == RobotConstants.RunType.TELEOP_WITH_EMBEDDED_AUTONOMOUS) {
                configXPath = configXML.getPath("TELEOP_SETTINGS");
                String logging_level = configXPath.getRequiredTextInRange("log_level", configXPath.validRange("d", "v", "vv", "off"));
                double driveTrainVelocityHigh = configXPath.getRequiredDouble("drive_train_velocity/high");
                double driveTrainVelocityMedium = configXPath.getRequiredDouble("drive_train_velocity/medium");
                double driveTrainVelocityLow = configXPath.getRequiredDouble("drive_train_velocity/low");

                teleOpSettings = new TeleOpSettings(logging_level,
                        driveTrainVelocityHigh, driveTrainVelocityMedium, driveTrainVelocityLow);
                RobotLogCommon.c(TAG, "TeleOp configuration: log level " + teleOpSettings.logLevel);
            } else
                teleOpSettings = null;

            // Get the drive train configuration.
            // Check for the special run types AUTO_NO_DRIVE
            // and TELEOP_NO_DRIVE, which we use for stand-alone
            // testing of devices and the camera.
            if (!(pRunType == RobotConstants.RunType.AUTO_NO_DRIVE ||
                    pRunType == RobotConstants.RunType.TELEOP_NO_DRIVE ||
                    pRunType == RobotConstants.RunType.TELEOP_NO_DRIVE_WITH_EMBEDDED_AUTONOMOUS)) {
                configXPath = configXML.getPath("DRIVE_TRAIN");
                driveTrain = new DriveTrain(hardwareMap, configXPath);
            } else
                driveTrain = null;

            // Only look at including cameras if the configuration needs them.
            if (!(pRunType == RobotConstants.RunType.AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_WITH_EMBEDDED_AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_NO_DRIVE_WITH_EMBEDDED_AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_OPENCV_CALIBRATION)) {
                configuredWebcams = null;
            } else {
                // Any configured VisionPortal webcams?
                configXPath = configXML.getPath("VISION_PORTAL_WEBCAM");
                String webcamYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
                RobotLogCommon.c(TAG, "VisionPortal webcam configuration option: " + webcamYesNo);

                if (webcamYesNo.equals("yes")) {
                    configuredWebcams = configXML.getConfiguredWebcams();
                    if (configuredWebcams.size() > 2)
                        throw new AutonomousRobotException(TAG, "CenterStage season: only two webcams at mnost are supported");

                    matchHardwareWebcamsWithConfiguredWebcams();
                }
                else
                    configuredWebcams = new EnumMap<>(RobotConstantsCenterStage.InternalWebcamId.class);
            }

            if (pRunType == RobotConstants.RunType.AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_WITH_EMBEDDED_AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_NO_DRIVE_WITH_EMBEDDED_AUTONOMOUS) {
                BasicIMU basicIMU = new BasicIMU(hardwareMap);
                imuReader = new IMUReader(basicIMU.getInitializedIMU());
            } else
                imuReader = null;

        } catch (ParserConfigurationException | SAXException | XPathExpressionException |
                 IOException ex) {
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException " + eMessage);
        }
    }

    // The FTC SDK uses a string such as "Webcam 1" to connect
    // a webcam via the HardwareMap and to create a WebcamName
    // object. Use the webcam's serial number in its WebcamName
    // object to associate the webcam with its counterpart in
    // RobotConfig.xml.
    private void matchHardwareWebcamsWithConfiguredWebcams() {
        String webcamId;
        for (int i = 1; i <= configuredWebcams.size(); i++) {
            webcamId = "Webcam " + new DecimalFormat("0").format(i);
            WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamId);
            if (!webcamName.isWebcam() || !webcamName.isAttached())
                throw new AutonomousRobotException(TAG, "Webcam " + webcamId +
                        " is not a webcam or is not attached");

            Optional<VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcam = configuredWebcams.values().stream()
                    .filter(webcam -> webcam.serialNumber.equals(webcamName.getSerialNumber().getString()))
                    .findFirst();

            if (!configuredWebcam.isPresent())
                throw new AutonomousRobotException(TAG,
                        "No configured webcam for serial number: " + webcamName.getSerialNumber());

            // Now that we have the correct association, add the WebcamName to the
            // configured camera.
            VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcamObject = configuredWebcam.get();
            configuredWebcamObject.setWebcamName(webcamName);
            RobotLogCommon.i(TAG, "Webcam hardware device " + webcamId +
                    " is associated by serial number " + webcamName.getSerialNumber() +
                    " with configured webcam " + configuredWebcamObject.internalWebcamId);
        }
    }

    // Fields captured from RobotConfig.xml.
    public static class TeleOpSettings {
        public final Level logLevel;
        public final double driveTrainVelocityHigh, driveTrainVelocityMedium, driveTrainVelocityLow;

        public TeleOpSettings(String pLogLevel,
                              double pDriveTrainVelocityHigh, double pDriveTrainVelocityMedium, double pDriveTrainVelocityLow) {
            switch (pLogLevel) {
                case "off": {
                    logLevel = Level.OFF;
                    break;
                }
                case "d": {
                    logLevel = Level.FINE;
                    break;
                }
                case "v": {
                    logLevel = Level.FINER;
                    break;
                }
                case "vv": {
                    logLevel = Level.FINEST;
                    break;
                }
                default: {
                    throw new AutonomousRobotException(TAG, "Invalid log level for TeleOp");
                }
            }

            driveTrainVelocityHigh = pDriveTrainVelocityHigh;
            driveTrainVelocityMedium = pDriveTrainVelocityMedium;
            driveTrainVelocityLow = pDriveTrainVelocityLow;
        }
    }

}

