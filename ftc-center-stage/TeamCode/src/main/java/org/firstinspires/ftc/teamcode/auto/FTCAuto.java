package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLCenterStage;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeoutException;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

public class FTCAuto {

    private static final String TAG = FTCAuto.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final String workingDirectory;

    private final RobotActionXMLCenterStage actionXML;
    private boolean keepCamerasRunning = false;
    private final VisionPortalWebcam visionPortalWebcam;
    private RobotConstantsCenterStage.InternalWebcamId webcamInUse;

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstants.RunType pRunType)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException, TimeoutException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        alliance = pAlliance;
        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        // Read the robot action file for all OpModes.
        actionXML = new RobotActionXMLCenterStage(xmlDirectory);

        //**TODO Replace with TeamProp XML/recognition
        //  Read the signal sleeve image recognition parameters from an xml file.
        //SignalSleeveParametersXML signalSleeveParametersXML = new SignalSleeveParametersXML(xmlDirectory);
        // signalSleeveParameters = signalSleeveParametersXML.getSignalSleeveParameters();
        //signalSleeveRecognition = new SignalSleeveRecognition(alliance);

        //**TODO assume 1 webcam for now.
        webcamInUse = robot.visionPortalWebcamConfiguration.webcams.get(0).webcamId;
        visionPortalWebcam = new VisionPortalWebcam(robot);
        visionPortalWebcam.enableWebcamFrameProcessor();

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    // In order to take multiple pictures from an Autonomous OpMode running
    // under TeleOp it is necessary *not* to shut down the camera(s) at the end of
    // runOpMode.
    public void runRobotWithCameras(RobotConstantsCenterStage.OpMode pOpMode) throws Exception {
        keepCamerasRunning = true;
        runRobot(pOpMode);
    }

    public void runRobot(RobotConstantsCenterStage.OpMode pOpMode) throws Exception {
        // In order for finally() to run we need to have a catch block below.
        // Without it the FTC runtime, which must have an UncaughtExceptionHandler,
        // prevents our finally() from running.
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs.
            // 12/28/2022 From the FTC SDK documentation: "whether the OpMode is
            // currently active. If this returns false, you should break out of
            // the loop in your runOpMode() method and return to its caller.
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runRobot()");
                return;
            }

            RobotLogCommon.i(TAG, "FTCAuto runRobot()");

            // Extract data from
            // the parsed XML file for the selected OpMode only.
            RobotActionXMLCenterStage.RobotActionDataCenterStage actionData = actionXML.getOpModeData(pOpMode.toString());

            Level logLevel = actionData.logLevel;
            if (logLevel != null) // null means use the default
                RobotLogCommon.setMostDetailedLogLevel(logLevel);
            RobotLogCommon.c(TAG, "Most detailed log level " + RobotLogCommon.getMostDetailedLogLevel());

            // Follow the choreography specified in the robot action file.
            // Note that executeAction returns false as a signal to stop
            // all processing immediately.
            List<RobotXMLElement> actions = actionData.actions;
            for (RobotXMLElement action : actions) {

                if (!linearOpMode.opModeIsActive())
                    return; // better to just bail out

                if (!executeAction(action))
                    return;
            }
        } finally {
            if (!keepCamerasRunning) {
                RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                visionPortalWebcam.finalShutdown();
            }
        }

        RobotLogCommon.i(TAG, "Exiting FTCAuto");
        linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
        linearOpMode.telemetry.update();
    }

    //===============================================================================================
    //===============================================================================================

    // Using the XML elements and attributes from the configuration file
    // RobotAction.xml, execute the action.
    @SuppressLint("DefaultLocale")
    private boolean executeAction(RobotXMLElement pAction) throws Exception {

        // Set up XPath access to the current action.
        XPathAccess actionXPath = new XPathAccess(pAction);
        String actionName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto action " + actionName);

        switch (actionName) {
            case "STOP_WEBCAM_STREAMING": {
                if (webcamInUse == RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS)
                    throw new AutonomousRobotException(TAG, "No webcam is active");

                visionPortalWebcam.stopStreaming();
                webcamInUse = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;
                break;
            }

            case "RESUME_WEBCAM_STREAMING": {
                if (webcamInUse == RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS)
                    throw new AutonomousRobotException(TAG, "No webcam is active");

                visionPortalWebcam.resumeStreaming();
                webcamInUse = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;
                break;
            }

            //**TODO
            // ENABLE_WEBCAM_FRAME_PROCESSOR
            // DISABLE_WEBCAM_FRAME_PROCESSOR
            // ENABLE_APRILTAG_PROCESSOR
            // DISABLE_APRILTAG_PROCESSOR

            // For testing, get a frame from a webcam managed by the VisionPortal
            // API and write it out to a file. Assume that the webcam has already
            // been started.
            case "TAKE_PICTURE_WEBCAM": {
                visionPortalWebcam.enableWebcamFrameProcessor();
                Pair<Mat, Date> image = visionPortalWebcam.getVisionPortalWebcamData(2000);

                if (image == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from " + webcamInUse);
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from " + webcamInUse);
                    linearOpMode.telemetry.update();
                } else {
                    RobotLogCommon.d(TAG, "Took a picture with " + webcamInUse);
                    String fileDate = TimeStamp.getDateTimeStamp(image.second);
                    String outputFilenamePreamble = workingDirectory + RobotConstants.imageDir + webcamInUse + "_" + fileDate;

                    String imageFilename = outputFilenamePreamble + "_IMG.png";
                    RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                    Imgcodecs.imwrite(imageFilename, image.first);

                    RobotLogCommon.d(TAG, "Image width " + image.first.cols() + ", height " + image.first.rows());
                    linearOpMode.telemetry.addData("Take picture with webcam:", "successful");
                    linearOpMode.telemetry.update();
                }

                break;
            }

            //**TODO replace Analyze the image on the signal sleeve.
            case "FIND_TEAM_PROP": {
                //Callable<RobotConstantsPowerPlay.SignalSleeveLocation> callableAnalyzeSignalSleeve =
                //        analyze_signal_sleeve(pAction, actionXPath);
                //callableAnalyzeSignalSleeve.call(); // execute now
                break;
            }

            case "SLEEP": { // I want sleep :)
                int sleepMs = actionXPath.getRequiredInt("ms");
                sleepInLoop(sleepMs);
                break;
            }

            // In testing this gives us a way to short-circuit a set
            //  of actions without commenting out any XML.
            // Shut down background threads, including the imu and the logger.
            case "STOP": {
                RobotLogCommon.closeLog();
                sleep(1000);
                return false;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the action " + actionName);
            }
        }

        // Action completed normally
        return true;
    }

    // Sleeps but also tests if the OpMode is still active.
    // Telemetry should keep the FTC runtime from shutting us down due to inactivity.
    private void sleepInLoop(int pMilliseconds) {
        RobotLogCommon.d(TAG, "Sleep for " + pMilliseconds + " milliseconds");
        linearOpMode.telemetry.addData("Sleeping for: ", "%d", pMilliseconds);
        linearOpMode.telemetry.update();

        int numSleeps = pMilliseconds / 100;
        int sleepRemainder = pMilliseconds % 100;
        int msSlept = 0;
        for (int i = 0; i < numSleeps; i++) {
            if (!linearOpMode.opModeIsActive())
                return;
            linearOpMode.sleep(100);
            msSlept += 100;

            // Only update telemetry every 500ms
            if (msSlept % 500 == 0) {
                linearOpMode.telemetry.addData("Slept for: ", "%d", msSlept);
                linearOpMode.telemetry.update();
            }
        }

        if (linearOpMode.opModeIsActive() && sleepRemainder != 0)
            linearOpMode.sleep(sleepRemainder);
    }

}

