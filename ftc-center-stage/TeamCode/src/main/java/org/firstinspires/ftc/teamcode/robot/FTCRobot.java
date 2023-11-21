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
import org.firstinspires.ftc.teamcode.robot.device.imu.GenericIMU;
import org.firstinspires.ftc.teamcode.robot.device.imu.IMUDirect;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.servo.DroneLauncherServo;
import org.firstinspires.ftc.teamcode.robot.device.servo.DualSPARKMiniController;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelIO;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelIOHolderServo;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
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
        ELEVATOR_LEFT, ELEVATOR_RIGHT, BOOM,
        MOTOR_ID_NPOS // for error checking
    }

    private final HardwareMap hardwareMap;

    public final TeleOpSettings teleOpSettings;
    public final DriveTrain driveTrain;

    // Instantiate here in FTCRobot so that these objects
    // can be shared between TeleOp and FTCAuto when it is
    // embedded within TeleOp.
    public final Elevator elevator;
    public final DualMotorMotion elevatorMotion;
    public final DualSPARKMiniController pixelIO;
    public final PixelIOHolderServo pixelIOHolderServo;
    public final PixelStopperServo pixelStopperServo;

    public final DroneLauncherServo droneLauncherServo;

    public final EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams;

    public final IMUDirect imuDirect;

    public FTCRobot(LinearOpMode pLinearOpMode, RobotConstants.RunType pRunType) throws InterruptedException {
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
            if (pRunType == RobotConstants.RunType.TELEOP || pRunType == RobotConstants.RunType.TELEOP_WITH_EMBEDDED_AUTONOMOUS ||
                    pRunType == RobotConstants.RunType.TELEOP_VISION_PREVIEW) {
                configXPath = configXML.getPath("TELEOP_SETTINGS");
                String logging_level = configXPath.getRequiredTextInRange("log_level", configXPath.validRange("d", "v", "vv", "off"));
                double driveTrainPowerHigh = configXPath.getRequiredDouble("drive_train_power/high");
                double driveTrainPowerLow = configXPath.getRequiredDouble("drive_train_power/low");

                teleOpSettings = new TeleOpSettings(logging_level, driveTrainPowerHigh, driveTrainPowerLow);
                RobotLogCommon.c(TAG, "TeleOp configuration: log level " + teleOpSettings.logLevel);
            } else
                teleOpSettings = null;

            // If you have a test setup without a drive train,
            // such as a Robot Controller and a webcam, then just
            // configure the drive train out in RobotConfig.xml.
            configXPath = configXML.getPath("DRIVE_TRAIN");
            String driveTrainYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            RobotLogCommon.c(TAG, "Drive train configuration option: " + driveTrainYesNo);
            if (driveTrainYesNo.equals("yes"))
                driveTrain = new DriveTrain(hardwareMap, configXPath);
            else
                driveTrain = null;

            // Get the configuration for the dual-motor elevator.
            configXPath = configXML.getPath("ELEVATOR");
            String elevatorInConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (elevatorInConfiguration.equals("yes")) {
                elevator = new Elevator(hardwareMap, configXPath);
                elevatorMotion = new DualMotorMotion(pLinearOpMode, elevator);
            } else {
                elevator = null;
                elevatorMotion = null;
            }

            // Get the configuration for pixel intake/outtake.
            configXPath = configXML.getPath("PIXEL_IO");
            String pixelIOInConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (pixelIOInConfiguration.equals("yes")) {
                pixelIO = new PixelIO(hardwareMap, configXPath);
            } else {
                pixelIO = null;
            }

            // Get the configuration for the pixel IO holder servo.
            configXPath = configXML.getPath("PIXEL_IO_HOLDER");
            String holderInConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (holderInConfiguration.equals("yes")) {
                pixelIOHolderServo = new PixelIOHolderServo(hardwareMap, configXPath);
            } else {
                pixelIOHolderServo = null;
            }

            // Get the configuration for the pixel stopper servo.
            configXPath = configXML.getPath("PIXEL_STOPPER");
            String stopperConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (stopperConfiguration.equals("yes")) {
                pixelStopperServo = new PixelStopperServo(hardwareMap, configXPath);
            } else {
                pixelStopperServo = null;
            }

            // Get the configuration for the pixel stopper servo.
            configXPath = configXML.getPath("DRONE_LAUNCHER");
            String launcherConfiguration = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            if (launcherConfiguration.equals("yes")) {
                droneLauncherServo = new DroneLauncherServo(hardwareMap, configXPath);
            } else {
                droneLauncherServo = null;
            }

            // In a competition the webcam(s) would be configured in and
            // used in Autonomous but not in TeleOp so we can't just check
            // the configuration file.
            if (pRunType == RobotConstants.RunType.TELEOP) {
                configuredWebcams = null;
            } else {
                // Any configured VisionPortal webcams?
                EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcamsLocal;
                configXPath = configXML.getPath("VISION_PORTAL_WEBCAM");
                String webcamYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
                RobotLogCommon.c(TAG, "VisionPortal webcam configuration option: " + webcamYesNo);

                if (webcamYesNo.equals("yes")) {
                    configuredWebcamsLocal = configXML.getConfiguredWebcams();
                    RobotLogCommon.d(TAG, "Number of webcams configured " + configuredWebcamsLocal.size());
                    if (configuredWebcamsLocal.size() > 2)
                        throw new AutonomousRobotException(TAG, "CenterStage season: only two webcams at mnost are supported");

                    matchHardwareWebcamsWithConfiguredWebcams(configuredWebcamsLocal);
                } else
                    configuredWebcamsLocal = null;

                configuredWebcams = configuredWebcamsLocal; // needed to preserve "final"
            }

            // In a competition the IMU would be configured in and
            // used in Autonomous but not in TeleOp.
            if (pRunType == RobotConstants.RunType.TELEOP)
                imuDirect = null;
            else {
                // Send XPath access in to GenericIMU; parse out logo and usb directions.
                configXPath = configXML.getPath("IMU");
                GenericIMU genericIMU = new GenericIMU(hardwareMap, configXPath);
                imuDirect = new IMUDirect(genericIMU.getImu());
            }
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
    private void matchHardwareWebcamsWithConfiguredWebcams(EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> pConfiguredWebcams) {
        String webcamId;
        for (int i = 1; i <= pConfiguredWebcams.size(); i++) {
            webcamId = "Webcam " + new DecimalFormat("0").format(i);
            WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamId);
            if (!webcamName.isWebcam() || !webcamName.isAttached())
                throw new AutonomousRobotException(TAG, "Webcam " + webcamId +
                        " is not a webcam or is not attached");

            //RobotLogCommon.d(TAG, "Webcam hardware device " + webcamId +
            //        " is attached to webcam serial number " + webcamName.getSerialNumber());

            Optional<VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcam = pConfiguredWebcams.values().stream()
                    .filter(webcam -> webcam.serialNumber.equals(webcamName.getSerialNumber().toString()))
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
        public final double driveTrainPowerHigh, driveTrainPowerLow;

        public TeleOpSettings(String pLogLevel,
                              double pDriveTrainPowerHigh, double pDriveTrainPowerLow) {
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

            driveTrainPowerHigh = pDriveTrainPowerHigh;
            driveTrainPowerLow = pDriveTrainPowerLow;
        }
    }

}

