package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.Threading;
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
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.AprilTagNavigation;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainMotion;
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
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

public class FTCAuto {

    private static final String TAG = FTCAuto.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final String workingDirectory;

    private final RobotActionXMLCenterStage actionXML;
    private double desiredHeading = 0.0; // always normalized

    private final DriveTrainMotion driveTrainMotion;
    private CompletableFuture<Void> asyncStraight;
    private CompletableFuture<Double> asyncTurn;

    private boolean keepIMUAndCamerasRunning = false;

    // Image recognition.
    private final TeamPropParameters teamPropParameters;
    private final TeamPropRecognition teamPropRecognition;
    EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions;
    private List<RobotXMLElement> teamPropLocationInsert;
    private boolean executeTeamPropLocationActions = false;

    private AprilTagNavigation aprilTagNavigation;

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstants.RunType pRunType)
            throws ParserConfigurationException, SAXException, XPathException, IOException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        alliance = pAlliance;
        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Read the robot action file for all OpModes.
        actionXML = new RobotActionXMLCenterStage(xmlDirectory);

        // Initialize the hardware and classes that control motion.
        // Do not initialize if the components have been configured out.
        if (robot.driveTrain != null)
            driveTrainMotion = new DriveTrainMotion(linearOpMode, robot);
        else
            driveTrainMotion = null;

        // Read the parameters for team prop recognition from the xml file.
        TeamPropParametersXML teamPropParametersXML = new TeamPropParametersXML(xmlDirectory);
        teamPropParameters = teamPropParametersXML.getTeamPropParameters();
        teamPropRecognition = new TeamPropRecognition(pAlliance);

        //!! 10/9/2023 Very important - I made an enquiry on the FTC Forum about Driver
        // Station camera streams when more than one webcam is in the configuration.
        // The answer came back that second camera's stream is the one that is displayed
        // during init. So start the AprilTag camera first and the Team Prop camera
        // second.
        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            // If the rear-facing webcam is in the configuration start it now with
            // its processor(s) disabled. It may be configured out during debugging.
            VisionPortalWebcamConfiguration.ConfiguredWebcam rearWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.REAR_WEBCAM);
            if (rearWebcamConfiguration != null) {
                VisionPortalWebcam visionPortalRearWebcam = new VisionPortalWebcam(rearWebcamConfiguration);
                rearWebcamConfiguration.setVisionPortalWebcam(visionPortalRearWebcam);
                //**TODO 10/17/23 temp visionPortalRearWebcam.setManualExposure(6, 250, 1000); // Use low exposure time to reduce motion blur
            }

            // Since the first task in Autonomous is to find the Team Prop, start the front webcam
            // with the processor for raw frames. The only time this camera might not be in the
            // configuration is during testing.
            VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
            if (frontWebcamConfiguration != null) {
                VisionPortalWebcam visionPortalFrontWebcam = new VisionPortalWebcam(Objects.requireNonNull(frontWebcamConfiguration));
                frontWebcamConfiguration.setVisionPortalWebcam(visionPortalFrontWebcam);
                visionPortalFrontWebcam.enableProcessor(RobotConstantsCenterStage.ProcessorIdentifier.WEBCAM_FRAME);
            }
        }

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    // In order to take multiple pictures from an Autonomous OpMode running
    // under TeleOp it is necessary *not* to shut down the camera(s) at the end of
    // runOpMode.
    public void runRobotWithCameras(RobotConstantsCenterStage.OpMode pOpMode) throws Exception {
        keepIMUAndCamerasRunning = true;
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
            RobotLogCommon.i(TAG, "IMU heading at start " + robot.imuReader.getIMUHeading());

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
            teamPropLocationActions = actionData.teamPropLocationActions;

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

                // Takes care of the case where the TEAM_PROP_LOCATION_CHOICE
                // action is the last action for the opmode in RobotConfig.xml.
                if (executeTeamPropLocationActions) { // any steps specific to the team prop location?
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
            if (!keepIMUAndCamerasRunning) {
                // Shut down the IMU and the cameras. This is the normal path for an Autonomous run.
                if (robot.imuReader != null) { // if the IMU is configured in
                    RobotLogCommon.i(TAG, "In FTCAuto finally: close the imu reader");
                    robot.imuReader.stopIMUReader();
                }

                if (robot.configuredWebcams != null) { // if webcam(s) are configured in
                    RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                    robot.configuredWebcams.forEach((k, v) -> v.getVisionPortalWebcam().finalShutdown());
                }
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
            // The robot moves without rotation in a direction relative to
            // the robot's current heading according to the "angle" parameter.
            case "STRAIGHT_BY": {
                straight_by(actionXPath, () -> {
                    try {
                        return actionXPath.getRequiredDouble("angle");
                    } catch (XPathExpressionException e) {
                        String eMessage = e.getMessage() == null ? "**no error message**" : e.getMessage();
                        throw new AutonomousRobotException(TAG, "XPath exception " + eMessage);
                    }
                }).call();
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "FORWARD": {
                straight_by(actionXPath, () -> 0.0).call();
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "BACK": {
                straight_by(actionXPath, () -> -180.0).call();
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "STRAFE_LEFT": {
                straight_by(actionXPath, () -> 90.0).call();
                break;
            }

            // Specialization of STRAIGHT_BY.
            case "STRAFE_RIGHT": {
                straight_by(actionXPath, () -> -90.0).call();
                break;
            }

            // Drive straight asynchronously.
            // The lambda gets the angle from the XML file.
            case "ASYNC_STRAIGHT_BY": {
                async_straight_by(actionXPath,
                        () -> {
                            try {
                                return actionXPath.getRequiredDouble("angle");
                            } catch (XPathExpressionException e) {
                                String eMessage = e.getMessage() == null ? "**no error message**" : e.getMessage();
                                throw new AutonomousRobotException(TAG, "XPath exception " + eMessage);
                            }
                        });
                break;
            }

            // Specialization of ASYNC_STRAIGHT_BY.
            // The lambda supplies the angle 0.0 (forward).
            case "ASYNC_FORWARD": {
                async_straight_by(actionXPath, () -> 0.0);
                break;
            }

            // Specialization of ASYNC_STRAIGHT_BY.
            case "ASYNC_BACK": {
                async_straight_by(actionXPath, () -> -180.0);
                break;
            }

            // Specialization of ASYNC_STRAIGHT_BY.
            case "ASYNC_STRAFE_LEFT": {
                async_straight_by(actionXPath, () -> 90.0);
                break;
            }

            // Specialization of ASYNC_STRAIGHT_BY.
            case "ASYNC_STRAFE_RIGHT": {
                async_straight_by(actionXPath, () -> -90.0);
                break;
            }

            // Making the robot turn ---
            // An XML attribute determines whether the angle of a turn is
            // relative to the starting position of the robot (heading 0.0)
            // or the current position of the robot. For example, if the
            // current heading of the robot is 30 degrees (counter-clockwise)
            // a turn of -30 degrees relative to the starting position of
            // the robot will turn clockwise until the heading reaches -30
            // degrees (clockwise). But the same -30 angle relative to the
            // current position of the robot will turn clockwise until the
            // robot reaches a heading of 0.0.

            // At the same time, a turn may be normalized or unnormalized. A
            // normalized turn is always the shortest distance from the
            // current heading to the desired heading. For example, if the
            // current heading is 30 degrees and the requested angle is -210
            // degrees (clockwise) to reach the desired heading of -180 degrees,
            // the robot will turn counter-clockwise 150 degrees. If the turn
            // is unnormalized, the robot will always turn in the requested
            // direction.
            case "TURN": {
                desiredHeading = turn(actionXPath).call();
                break;
            }

            case "ASYNC_TURN": {
                async_turn(actionXPath);
                break;
            }

            // Straighten out the robot by turning to the desired heading.
            case "DESKEW": {
                deskew();
                break;
            }

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
                RobotLogCommon.d(TAG, "Disabled processor " + processorIdString + " on webcam " + webcamIdString);
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
                spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS, Pair.create(new Rect(0, 0, 0, 0), team_prop_npos));
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
                } else {
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

            // For testing: just look for AprilTags.
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

            // Locate a specific AprilTag and drive the robot into position
            // in front of it.
            case "NAVIGATE_TO_APRIL_TAG": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);

                // If the internal id of the webcam in the current AprilTagNavigation object
                // does not match the id just specified then recreate the AprilTagNavigation object.
                if (aprilTagNavigation == null || aprilTagNavigation.getInternalWebcamId() !=
                        webcamId) {
                    RobotLogCommon.d(TAG, "Switching AprilTag navigation to " + webcamIdString);
                    VisionPortalWebcam visionPortalWebcam = Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                    aprilTagNavigation = new AprilTagNavigation(alliance, linearOpMode, robot, visionPortalWebcam);
                }

                int desiredTagId = actionXPath.getRequiredInt("desired_tag_id");
                double desiredDistanceFromTag = actionXPath.getRequiredDouble("desired_distance_from_tag");

                String directionString = actionXPath.getRequiredText("direction").toUpperCase();
                DriveTrainConstants.Direction direction =
                        DriveTrainConstants.Direction.valueOf(directionString);
                RobotLogCommon.d(TAG, "Navigating to AprilTag with id " + desiredTagId);
                RobotLogCommon.d(TAG, "Stop at " + desiredDistanceFromTag + " from the tag");
                RobotLogCommon.d(TAG, "Direction of travel " + directionString);
                if (!aprilTagNavigation.driveToAprilTag(desiredTagId, desiredDistanceFromTag, direction)) {
                   RobotLogCommon.d(TAG, "Navigation to AprilTag was not successful");
                   return false;
                }

                deskew(); // make sure the robot is aligned with the desired heading

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
                sleep(1000);
                return false;
            }

            // For testing: record the heading and pitch from the IMU for
            // 10 seconds.
            case "RECORD_IMU": {
                ElapsedTime imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                double heading;
                double pitch;
                double roll;
                //double baselinePitch = robot.imuReader.getIMUPitch();

                //RobotLogCommon.d(TAG, "IMU baseline pitch " + baselinePitch);
                //linearOpMode.telemetry.addData("IMU baseline pitch: ", baselinePitch);
                //linearOpMode.telemetry.update();

                //sleepInLoop(1000);
                imuTimer.reset();

                while (linearOpMode.opModeIsActive() && imuTimer.time() < 10000) {
                    heading = robot.imuReader.getIMUHeading();
                    pitch = robot.imuReader.getIMUPitch();
                    roll = robot.imuReader.getIMURoll();

                    String logString = "heading " + heading + ", pitch " + pitch + ", roll " + roll;
                    RobotLogCommon.d(TAG, "IMU " + logString);
                    linearOpMode.telemetry.addData("IMU ", logString);
                    linearOpMode.telemetry.update();

                    //if (Math.abs(pitch) > Math.abs(baselinePitch) + 2.5) {
                    //    linearOpMode.telemetry.addData("IMU pitch +- 2.5 deg from baseline: ", pitch);
                    //    linearOpMode.telemetry.update();
                    //}
                    sleep(500); // be careful - small sleep values flood the log
                }

                break;
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

    private void async_straight_by(XPathAccess pActionXPath, Supplier<Double> pAngle) throws XPathExpressionException, IOException, InterruptedException, TimeoutException {
        String operation = pActionXPath.getRequiredTextInRange("operation", pActionXPath.validRange("start", "wait"));
        switch (operation) {
            case "start": {
                Callable<Void> callableDriveToPosition =
                        straight_by(pActionXPath, pAngle);
                asyncStraight = Threading.launchAsync(callableDriveToPosition);
                break;
            }

            // Wait for the straight run to complete.
            case "wait": {
                if (asyncStraight == null)
                    throw new AutonomousRobotException(TAG, "In wait: asyncStraight has not been initalized");

                RobotLogCommon.d(TAG, "Async drive straight: wait");
                Threading.getFutureCompletion(asyncStraight);
                RobotLogCommon.d(TAG, "Async drive straight: complete");
                asyncStraight = null;
                break;
            }

            default:
                throw new AutonomousRobotException(TAG, "Invalid asynchronous straight operation: " + operation);
        }
    }

    // Return a Callable that can be used to move the robot directly or can be launched
    // to move the robot asynchronously.
    private Callable<Void> straight_by(XPathAccess pActionXPath, Supplier<Double> pAngle) throws XPathExpressionException {
        if (asyncStraight != null) // Prevent double initialization
            throw new AutonomousRobotException(TAG, "straight_by: asyncStraight is active");

        if (asyncTurn != null)
            throw new AutonomousRobotException(TAG, "straight_by: asyncTurn is active");

        // Start with the absolute value of the click count. The sign of the click count for each of the
        // motors will be adjusted depending on the sign of the angle.
        double distanceInches = Math.abs(pActionXPath.getRequiredDouble("distance"));
        RobotLogCommon.d(TAG, "Drive " + distanceInches + " inches" + " at angle " + pAngle.get());
        int targetClicks = (int) (distanceInches * robot.driveTrain.getClicksPerInch());

        // Default to no ramp-down.
        double rampDownInches = Math.abs(pActionXPath.getDouble("ramp_down_at_distance_remaining", 0.0));
        int rampDownAtClicksRemaining = (int) (rampDownInches * robot.driveTrain.getClicksPerInch());

        // Sanity check.
        if (rampDownAtClicksRemaining > targetClicks)
            rampDownAtClicksRemaining = targetClicks;

        // The velocity factor must be positive and in the range > 0.0 and <= 1.0
        double velocity = Math.abs(pActionXPath.getRequiredDouble("velocity")); // fraction of maximum
        if (velocity > 1.0 || velocity < DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);

        int finalRampDownAtClicksRemaining = rampDownAtClicksRemaining;

        return () -> {
            driveTrainMotion.straight(targetClicks, pAngle.get(), velocity, finalRampDownAtClicksRemaining, desiredHeading);
            return null;
        };
    }

    public void async_turn(XPathAccess pActionXPath) throws XPathExpressionException, IOException, InterruptedException, TimeoutException {
        String operation = pActionXPath.getRequiredTextInRange("operation", pActionXPath.validRange("start", "wait"));
        switch (operation) {
            case "start": {
                Callable<Double> callableTurn = turn(pActionXPath);
                asyncTurn = Threading.launchAsync(callableTurn);
                break;
            }

            // Wait for the turn to complete.
            case "wait": {
                if (asyncTurn == null)
                    throw new AutonomousRobotException(TAG, "In wait: asyncTurn has not been initalized");

                RobotLogCommon.d(TAG, "Async turn: wait");
                desiredHeading = Threading.getFutureCompletion(asyncTurn);
                RobotLogCommon.d(TAG, "Async turn: complete");
                asyncTurn = null;
                break;
            }

            default:
                throw new AutonomousRobotException(TAG, "Invalid asynchronous turn operation: " + operation);
        }
    }

    // Return a Callable that can be used to turn the robot directly or can be launched
    // to turn the robot asynchronously.
    private Callable<Double> turn(XPathAccess pActionXPath) throws XPathExpressionException, IOException, InterruptedException, TimeoutException {
        if (asyncTurn != null) // Prevent double initialization
            throw new AutonomousRobotException(TAG, "turn: asyncTurn is active");

        if (asyncStraight != null)
            throw new AutonomousRobotException(TAG, "turn: asyncStraight is active");

        String postTurnString = pActionXPath.getTextInRange("@post_turn_heading", "relative_to_current", pActionXPath.validRange("relative_to_start", "relative_to_current")).toUpperCase();
        DriveTrainConstants.PostTurnHeading postTurnHeading = DriveTrainConstants.PostTurnHeading.valueOf(postTurnString);

        String normalizationString = pActionXPath.getTextInRange("@turn_normalization", "normalized", pActionXPath.validRange("normalized", "unnormalized")).toUpperCase();
        DriveTrainConstants.TurnNormalization turnNormalization = DriveTrainConstants.TurnNormalization.valueOf(normalizationString);

        double angle = pActionXPath.getRequiredDouble("angle");

        // If the requested post-turn heading is RELATIVE_TO_START then we have to
        // adjust the angle. So if the requested angle is -90 and the current
        // heading is 30 then the turn will be -120.
        double currentHeading = robot.imuReader.getIMUHeading();
        if (postTurnHeading == DriveTrainConstants.PostTurnHeading.RELATIVE_TO_START) {
            angle = angle - currentHeading;
            RobotLogCommon.d(TAG, "Turn is relative to the starting position of the robot");
        } else
            RobotLogCommon.d(TAG, "Turn is relative to the current heading of the robot");

        switch (turnNormalization) {
            case NORMALIZED: {
                RobotLogCommon.d(TAG, "TURN normalized, angle " + angle);
                if (!((angle >= 0 && angle < 180) || (angle <= 0 && angle >= -180)))
                    throw new AutonomousRobotException(TAG, "Normalized angle out of range " + angle);
                break;
            }
            case UNNORMALIZED: {
                RobotLogCommon.d(TAG, "TURN unnormalized, angle " + angle);
                if (!((angle >= 0 && angle < 360) || (angle <= 0 && angle > -360)))
                    throw new AutonomousRobotException(TAG, "Unnormalized angle out of range " + angle);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized turn normalization " + turnNormalization);
        }

        double power = Math.abs(pActionXPath.getRequiredDouble("power")); // fraction of maximum
        if (power > 1.0 || power < DriveTrainConstants.MINIMUM_TURN_POWER)
            throw new AutonomousRobotException(TAG, "Power out of range " + power);

        // Default to no ramp-down. For consistency always make the rampdown value positive.
        double rampDownAtAngleRemaining = Math.abs(pActionXPath.getDouble("ramp_down_at_angle_remaining", 0.0));
        RobotLogCommon.d(TAG, "TURN ramp down " + rampDownAtAngleRemaining);

        // Sanity check.
        if (rampDownAtAngleRemaining > Math.abs(angle)) {
            rampDownAtAngleRemaining = Math.abs(angle);
            RobotLogCommon.d(TAG, "TURN ramp down reset to " + rampDownAtAngleRemaining);
        }

        final double finalRampDownAtAngleRemaining = rampDownAtAngleRemaining; // reqired for lambda
        final double finalAngle = angle;
        return () -> driveTrainMotion.turn(desiredHeading, robot.imuReader.getIMUHeading(), finalAngle, power, finalRampDownAtAngleRemaining, turnNormalization);
    }

    // Straighten out the robot by turning to the desired heading.
    // Make a normalized turn at minimum power.
    private void deskew() throws IOException, InterruptedException, TimeoutException {
        if (asyncStraight != null)
            throw new AutonomousRobotException(TAG, "Deskew not allowed while asyncStraight is in progress");

        if (asyncTurn != null)
            throw new AutonomousRobotException(TAG, "Deskew not allowed while asyncTurn is in progress");

        double currentHeading = robot.imuReader.getIMUHeading();
        RobotLogCommon.d(TAG, "Current heading " + currentHeading);
        double degreeDifference = DEGREES.normalize(desiredHeading - currentHeading);
        if (Math.abs(degreeDifference) >= DriveTrainConstants.SKEW_THRESHOLD_DEGREES) {
            RobotLogCommon.d(TAG, "Desired heading " + desiredHeading);
            RobotLogCommon.d(TAG, "De-skewing " + degreeDifference + " degrees");
            driveTrainMotion.turn(desiredHeading, currentHeading, 0.0, DriveTrainConstants.MINIMUM_TURN_POWER, 0.0, DriveTrainConstants.TurnNormalization.NORMALIZED);
        }
    }

    //## We noticed that the robot took time starting and stopping for
    // short distances (such as 1.0") at .3 velocity while at longer
    // distances we sometimes want to keep the low velocity.
    private double shortDistanceVelocity(double pDistance) {
        return Math.abs(pDistance) < 2.0 ? 0.5 : 0.3;
    }

    //## We noticed that the robot took time starting and stopping for
    // small angles (such as 3 degrees) at .3 velocity while at larger
    // angles we sometimes want to keep the low velocity.
    private double smallAngleVelocity(double pAngle) {
        return Math.abs(pAngle) < 5.0 ? 0.5 : 0.3;
    }
}

