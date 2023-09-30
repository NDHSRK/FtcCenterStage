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
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropReturn;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLCenterStage;
import org.firstinspires.ftc.teamcode.auto.xml.TeamPropParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamImageProvider;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeoutException;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

public class FTCAuto {

    private static final String TAG = FTCAuto.class.getSimpleName();

    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final String workingDirectory;

    private final RobotActionXMLCenterStage actionXML;
    private boolean keepCamerasRunning = false;

    // Image recognition.
    private final TeamPropParameters teamPropParameters;
    private final TeamPropRecognition teamPropRecognition;
    EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions;
    private List<RobotXMLElement> teamPropLocationInsert;
    private boolean executeTeamPropLocationActions = false;

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstants.RunType pRunType)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException, TimeoutException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Read the robot action file for all OpModes.
        actionXML = new RobotActionXMLCenterStage(xmlDirectory);

        // Read the parameters for team prop recognition from the xml file.
        TeamPropParametersXML teamPropParametersXML = new TeamPropParametersXML(xmlDirectory);
        teamPropParameters = teamPropParametersXML.getTeamPropParameters();
        teamPropRecognition = new TeamPropRecognition(pAlliance);

        // Since the first task in Autonomous is to find the Team Prop, start the front webcam
        // with the processor for raw frames.
        VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
        VisionPortalWebcam visionPortalFrontWebcam = new VisionPortalWebcam(Objects.requireNonNull(frontWebcamConfiguration));
        frontWebcamConfiguration.setVisionPortalWebcam(visionPortalFrontWebcam);
        visionPortalFrontWebcam.enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);

        // If the rear-facing webcam is in the configuration start it now with
        // its processor(s) disabled. It may not be configured in during debugging.
        VisionPortalWebcamConfiguration.ConfiguredWebcam rearWebcamConfiguration =
                robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.REAR_WEBCAM);
        if (rearWebcamConfiguration != null) {
            VisionPortalWebcam visionPortalRearWebcam = new VisionPortalWebcam(rearWebcamConfiguration);
            rearWebcamConfiguration.setVisionPortalWebcam(visionPortalRearWebcam);
        }

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

            // For the current OpMode get the actions for all three team prop locations.
            // We won't know which location to go for until we've performed image recognition.
            // This reference may be null if no team prop location actions have been defined
            // for an OpMode.
            // EnumMap< RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions;

            // Follow the choreography specified in the robot action file.
            // Note that executeAction returns false as a signal to stop
            // all processing immediately.
            List<RobotXMLElement> actions = actionData.actions;
            for (RobotXMLElement action : actions) {

                if (!linearOpMode.opModeIsActive())
                    return; // better to just bail out

                if (!executeTeamPropLocationActions) { // execute steps specific to team prop locations now?
                    // No, but executeAction may change that.
                    if (!executeAction(action))
                        return;
                }

                // Takes care of the case where the SIGNAL_SLEEVE_LOCATION_CHOICE
                // action is the last action for the opmode in RobotConfig.xml.
                if (executeTeamPropLocationActions) { // any steps specific to the signal sleeve location?
                    // Yes, execute all of those actions now.
                    for (RobotXMLElement insertedStep : teamPropLocationInsert) {
                        if (insertedStep.getRobotXMLElementName().equals("TEAM_PROP_LOCATION_CHOICE"))
                            throw new AutonomousRobotException(TAG, "Nesting of TEAM_PROP_LOCATION_CHOICE is not allowed");

                        if (!linearOpMode.opModeIsActive())
                            return; // better to just bail out

                        if (!executeAction(insertedStep))
                            return;
                    }
                    teamPropLocationInsert.clear();
                    executeTeamPropLocationActions = false;
                }
            }
        } finally {
            if (!keepCamerasRunning) {
                RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                robot.configuredWebcams.forEach((k,v) -> v.getVisionPortalWebcam().finalShutdown());
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
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().stopStreaming();
                RobotLogCommon.d(TAG, "Stopped streaming webcam " + webcamIdString);
                break;
            }

            case "RESUME_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().resumeStreaming();
                RobotLogCommon.d(TAG, "Resumed streaming webcam " + webcamIdString);
                break;
            }

            case "ENABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsCenterStage.ProcessorIdentifier processorId =
                        RobotConstantsCenterStage.ProcessorIdentifier.valueOf(processorIdString);
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().enableProcessor(processorId);
                RobotLogCommon.d(TAG, "Enabled processor " + processorIdString + " on webcam " + webcamIdString);
                break;
            }

            case "DISABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsCenterStage.ProcessorIdentifier processorId =
                        RobotConstantsCenterStage.ProcessorIdentifier.valueOf(processorIdString);
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().disableProcessor(processorId);
                RobotLogCommon.d(TAG, "Enabled processor " + processorIdString + " on webcam " + webcamIdString);
                break;
            }

            // For testing, get a frame from a webcam managed by the VisionPortal
            // API and write it out to a file. Assume that the webcam has already
            // been started.
            case "TAKE_PICTURE_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcam visionPortalWebcam = Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                visionPortalWebcam.enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);

                VisionPortalWebcamImageProvider provider = new VisionPortalWebcamImageProvider(visionPortalWebcam);
                Pair<Mat, Date> image = provider.getImage();
                if (image == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.update();
                } else {
                    RobotLogCommon.d(TAG, "Took a picture with " + webcamIdString);
                    String fileDate = TimeStamp.getDateTimeStamp(image.second);
                    String outputFilenamePreamble = workingDirectory + RobotConstants.IMAGE_DIR + webcamIdString + "_" + fileDate;

                    String imageFilename = outputFilenamePreamble + "_IMG.png";
                    RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                    Imgcodecs.imwrite(imageFilename, image.first);

                    RobotLogCommon.d(TAG, "Image width " + image.first.cols() + ", height " + image.first.rows());
                    linearOpMode.telemetry.addData("Take picture with webcam:", "successful");
                    linearOpMode.telemetry.update();
                }

                break;
            }

            // Find the location of the Team Prop.
            case "FIND_TEAM_PROP": {
                // Prepare for image recognition.
                VisionParameters.ImageParameters teamPropImageParameters =
                        actionXML.getImageParametersFromXPath(pAction, "image_parameters");
                String webcamIdString = teamPropImageParameters.image_source.toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);

                VisionPortalWebcam visionPortalWebcam = Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                visionPortalWebcam.enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);
                VisionPortalWebcamImageProvider imageProvider = new VisionPortalWebcamImageProvider(visionPortalWebcam);

                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredText("team_prop_recognition/recognition_path");
                RobotConstantsCenterStage.TeamPropRecognitionPath teamPropRecognitionPath =
                        RobotConstantsCenterStage.TeamPropRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + teamPropRecognitionPath);

                // Set the team prop recognition parameters for the current OpMode.
                EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows =
                        new EnumMap<>(RobotConstantsCenterStage.SpikeLocationWindow.class);

                // The left window onto the three spikes may be the leftmost spike or the
                // center spike. The right window is always immediately to the right of the
                // left window.
                // Get the boundaries for the left window onto the spikes.
                int left_x = actionXPath.getRequiredInt("team_prop_recognition/left_window/x");
                int left_y = actionXPath.getRequiredInt("team_prop_recognition/left_window/y");
                int left_width = actionXPath.getRequiredInt("team_prop_recognition/left_window/width");
                int left_height = actionXPath.getRequiredInt("team_prop_recognition/left_window/height");
                RobotConstantsCenterStage.TeamPropLocation team_prop_in_left_window = RobotConstantsCenterStage.TeamPropLocation.valueOf(actionXPath.getRequiredText("team_prop_recognition/left_window/prop_location").toUpperCase());
                spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.LEFT, Pair.create(new Rect(left_x, left_y, left_width, left_height), team_prop_in_left_window));

                // Get the boundaries for the right window onto the spikes.
                int right_width = actionXPath.getRequiredInt("team_prop_recognition/right_window/width");
                RobotConstantsCenterStage.TeamPropLocation team_prop_in_right_window = RobotConstantsCenterStage.TeamPropLocation.valueOf(actionXPath.getRequiredText("team_prop_recognition/right_window/prop_location").toUpperCase());
                // Note: the right window starts 1 pixel past the left element. The height of the right
                // window is the same as that of the left window.
                spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT, Pair.create(new Rect(left_x + left_width, left_y, right_width, left_height), team_prop_in_right_window));

                // Set the shipping hub level to infer if the Shipping Hub Element is either the left or right window.
                RobotConstantsCenterStage.TeamPropLocation team_prop_npos = RobotConstantsCenterStage.TeamPropLocation.valueOf(actionXPath.getRequiredText("team_prop_recognition/team_prop_npos/prop_location").toUpperCase());
                spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS, Pair.create(new Rect(0, 0, 0, 0),team_prop_npos));
                teamPropParameters.setSpikeWindows(spikeWindows);

                // Perform image recognition.
                TeamPropReturn teamPropReturn =
                        teamPropRecognition.recognizeTeamProp(imageProvider, teamPropImageParameters, teamPropParameters, teamPropRecognitionPath);

                RobotConstantsCenterStage.TeamPropLocation finalTeamPropLocation;

                if (teamPropReturn.recognitionResults == RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR ||
                   teamPropReturn.recognitionResults == RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL) {
                // Something went wrong during recognition but don't crash; use the default location of CENTER_SPIKE.
                finalTeamPropLocation = RobotConstantsCenterStage.TeamPropLocation.CENTER_SPIKE;
                RobotLogCommon.d(TAG, "Error in computer vision subsystem; using default location of CENTER_SPIKE");
            }
                else {
                    finalTeamPropLocation = teamPropReturn.teamPropLocation;
                    RobotLogCommon.d(TAG, "Team Prop Location " + teamPropReturn.teamPropLocation);
                    linearOpMode.telemetry.addData("Team Prop Location: ", teamPropReturn.teamPropLocation);
                    linearOpMode.telemetry.update();
                }

                // Prepare to execute the robot actions for the team prop location that was found.
                // If you're only testing team prop recognition then there may not be any associated
                // actions.
                if (teamPropLocationActions != null)
                    teamPropLocationInsert = new ArrayList<>(Objects.requireNonNull(teamPropLocationActions.get(finalTeamPropLocation)));
                break;
            }

            case "TEAM_PROP_LOCATION_CHOICE": {
                if (teamPropLocationInsert == null)
                    throw new AutonomousRobotException(TAG, "Missing element FIND_TEAM_PROP");

                executeTeamPropLocationActions = true;
                break;
            }

            case "FIND_APRIL_TAGS": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcam visionPortalWebcam = Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                List<AprilTagDetection> aprilTags = visionPortalWebcam.getAprilTagData(500);
                if (aprilTags.isEmpty()) {
                    linearOpMode.telemetry.addLine("No AprilTags found");
                    linearOpMode.telemetry.update();
                    RobotLogCommon.d(TAG, "No AprilTags found");
                } else telemetryAprilTag(aprilTags);
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

    // Copied from the sample ConceptAprilTag and slightly modified.
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag(List<AprilTagDetection> pCurrentDetections) {

        linearOpMode.telemetry.addData("# AprilTags Detected", pCurrentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : pCurrentDetections) {
            if (detection.metadata != null) {
                String detectionId = String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name);
                String XYZ = String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                String PRY = String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                String RBE = String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

                linearOpMode.telemetry.addLine(detectionId);
                linearOpMode.telemetry.addLine(XYZ);
                linearOpMode.telemetry.addLine(PRY);
                linearOpMode.telemetry.addLine(RBE);

                RobotLogCommon.d(TAG, detectionId);
                RobotLogCommon.d(TAG, XYZ);
                RobotLogCommon.d(TAG, PRY);
                RobotLogCommon.d(TAG, RBE);
            } else {
                String unknownId = String.format("\n==== (ID %d) Unknown", detection.id);
                String center = String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y);

                linearOpMode.telemetry.addLine(unknownId);
                linearOpMode.telemetry.addLine(center);
                RobotLogCommon.d(TAG, unknownId);
                RobotLogCommon.d(TAG, center);
            }
        }   // end for() loop

        // Add "key" information to telemetry
        linearOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        linearOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        linearOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        linearOpMode.telemetry.update();

        RobotLogCommon.d(TAG, "\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        RobotLogCommon.d(TAG, "PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        RobotLogCommon.d(TAG, "RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}

