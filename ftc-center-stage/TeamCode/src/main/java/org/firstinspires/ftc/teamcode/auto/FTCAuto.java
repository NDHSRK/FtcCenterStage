package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage.AprilTagId.getEnumValue;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;
import org.firstinspires.ftc.teamcode.auto.vision.BackdropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.CameraToCenterCorrections;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropReturn;
import org.firstinspires.ftc.teamcode.auto.xml.BackdropParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLCenterStage;
import org.firstinspires.ftc.teamcode.common.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.auto.xml.TeamPropParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.MultiPortalAuto;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Winch;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.AprilTagNavigation;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainMotion;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
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
    private PixelStopperServo.PixelServoState pixelServoState;
    private final String workingDirectory;
    private final int autoStartDelay;
    private final EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;
    private final RobotActionXMLCenterStage actionXML;
    private final EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowMapping> collectedSpikeWindowMapping;
    private final SpikeWindowMappingXML spikeWindowMappingXML;
    private SpikeWindowMapping currentSpikeWindowData;

    private RobotConstantsCenterStage.InternalWebcamId openWebcam = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;
    private double desiredHeading = 0.0; // always normalized
    private final DriveTrainMotion driveTrainMotion;
    private CompletableFuture<Void> asyncStraight;
    private CompletableFuture<Double> asyncTurn;

    private boolean keepCamerasRunning = false;

    // Image recognition.
    private final TeamPropParameters teamPropParameters;
    private final TeamPropRecognition teamPropRecognition;
    EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions;
    private List<RobotXMLElement> teamPropLocationInsert;
    private boolean executeTeamPropLocationActions = false;

    private final BackdropParameters backdropParameters;
    private AprilTagNavigation aprilTagNavigation;

    private CompletableFuture<Void> asyncMoveElevator;
    private Elevator.ElevatorLevel currentElevatorLevel = Elevator.ElevatorLevel.GROUND;

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstants.RunType pRunType)
            throws ParserConfigurationException, SAXException, XPathException, IOException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        alliance = pAlliance;
        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        // The initial state of the pixel stopper must be "held".
        // This will change to "released" before outtake.
        if (robot.pixelStopperServo != null) { // may be null during testing
            robot.pixelStopperServo.hold();
            pixelServoState = PixelStopperServo.PixelServoState.HOLD;
        }

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Get the configurable delay at Autonomous startup.
        autoStartDelay = robot.startParameters.autoStartDelay;

        // Get the ending positions of the robot for all competition
        // OpModes.
        autoEndingPositions = robot.startParameters.autoEndingPositions;

        // Read the RobotAction XXX.xml file for all OpModes.
        RobotLogCommon.c(TAG, "Getting the Autonomous choreographies from " + robot.startParameters.robotActionFilename);
        actionXML = new RobotActionXMLCenterStage(robot.startParameters.robotActionFilename);

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

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        spikeWindowMappingXML = new SpikeWindowMappingXML(robot.startParameters.robotActionFilename);
        collectedSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping();

        // Read the parameters for the backdrop from the xml file.
        BackdropParametersXML backdropParametersXML = new BackdropParametersXML(xmlDirectory);
        backdropParameters = backdropParametersXML.getBackdropParameters();

        // Start the front webcam with the webcam frame processor.
        // We can start a camera by using the <START_CAMERA> action in RobotAction.xml
        // but since the first task in Autonomous is to find the Team Prop, we save
        // time by starting the front webcam here with a processor for raw frames. The
        // only time this camera might not be in the configuration is during testing.
        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
            if (frontWebcamConfiguration != null) {
                VisionProcessor webcamFrameProcessor = new RawFrameProcessor.Builder().build();
                RawFrameWebcam rawFrameWebcam = new RawFrameWebcam(frontWebcamConfiguration,
                        Pair.create(RobotConstantsCenterStage.ProcessorIdentifier.RAW_FRAME, webcamFrameProcessor));
                if (!rawFrameWebcam.waitForWebcamStart(2000))
                    throw new AutonomousRobotException(TAG, "Unable to start front webcam");
                frontWebcamConfiguration.setVisionPortalWebcam(rawFrameWebcam);
                openWebcam = RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM;
            }
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
            robot.imuDirect.resetIMUYaw();
            RobotLogCommon.i(TAG, "IMU heading at start " + robot.imuDirect.getIMUHeading());

            // If the StartParameters.xml file contains a non-zero start delay
            // for Autonomous wait here.
            if (autoStartDelay != 0)
                sleepInLoop(autoStartDelay * 1000);

            // Get the spike window data for the current OpMode.
            // Note: if the current OpMode does not include an action of FIND_TEAM_PROP
            // then a lookup on the OpMode will return null. This is legal for such
            // OpModes as TEST_ELEVATOR.
            if (!collectedSpikeWindowMapping.isEmpty())
                currentSpikeWindowData = collectedSpikeWindowMapping.get(pOpMode);

            if (currentSpikeWindowData != null)
                RobotLogCommon.d(TAG, "Retrieved spike window data for OpMode " + pOpMode);
            else RobotLogCommon.d(TAG, "No spike window data for OpMode " + pOpMode);

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
                    if (!executeAction(action, pOpMode))
                        return;
                }

                // Takes care of the case where the TEAM_PROP_LOCATION_CHOICE
                // action is the last action for the opmode in RobotConfig.xml.
                if (executeTeamPropLocationActions) { // any steps specific to the team prop location?
                    // Yes, execute all of those actions now.
                    executeTeamPropLocationActions = false; // but only once
                    for (RobotXMLElement insertedStep : teamPropLocationInsert) {
                        if (insertedStep.getRobotXMLElementName().equals("TEAM_PROP_LOCATION_CHOICE"))
                            throw new AutonomousRobotException(TAG, "Nesting of TEAM_PROP_LOCATION_CHOICE is not allowed");

                        if (!linearOpMode.opModeIsActive())
                            return; // better to just bail out

                        if (!executeAction(insertedStep, pOpMode))
                            return;
                    }
                    teamPropLocationInsert.clear();
                }
            }
        } finally {
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.d(TAG, "FTCAuto OpMode not active in finally block");
                RobotLogCommon.i(TAG, "FTCAuto OpMode not active in finally block");
            } else {
                RobotLogCommon.i(TAG, "In FTCAuto finally block");

                failsafeElevator(); // bring the elevator to GROUND

           /*
            if (!keepCamerasRunning) {
                //**TODO Attempt at an orderly shutdown causes the Robot Controller to crash in easyopencv.
                if (robot.configuredWebcams != null) { // if webcam(s) are configured in
                    RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                    robot.configuredWebcams.forEach((k, v) -> {
                                if (v != null && v.getVisionPortalWebcam() != null)
                                    v.getVisionPortalWebcam().finalShutdown();
                            });
                }
            }
             */
            }
        }

        RobotLogCommon.i(TAG, "Exiting FTCAuto");
        linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
        linearOpMode.telemetry.update();
    }

    //===============================================================================================
    //===============================================================================================

    /**
     * @noinspection BooleanMethodIsAlwaysInverted
     */ // Using the XML elements and attributes from the configuration file
    // RobotAction.xml, execute the action.
    @SuppressLint("DefaultLocale")
    private boolean executeAction(RobotXMLElement pAction, RobotConstantsCenterStage.OpMode pOpMode) throws Exception {
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

            case "START_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);

                if (openWebcam != RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS)
                    throw new AutonomousRobotException(TAG, "Attempt to start webcam " + webcamId + " but " + openWebcam + " is open");

                openWebcam = webcamId;

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsCenterStage.ProcessorIdentifier processorId =
                        RobotConstantsCenterStage.ProcessorIdentifier.valueOf(processorIdString);

                switch (processorId) {
                    case RAW_FRAME: {
                        VisionProcessor webcamFrameProcessor = new RawFrameProcessor.Builder().build();
                        RawFrameWebcam rawFrameWebcam = new RawFrameWebcam(configuredWebcam,
                                Pair.create(processorId, webcamFrameProcessor));
                        configuredWebcam.setVisionPortalWebcam(rawFrameWebcam);
                        break;
                    }
                    case APRIL_TAG: {
                        VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                                // Follow the MultiPortal sample, which only includes setLensIntrinsics
                                .setLensIntrinsics(configuredWebcam.cameraCalibration.focalLengthX,
                                        configuredWebcam.cameraCalibration.focalLengthY,
                                        configuredWebcam.cameraCalibration.principalPointX,
                                        configuredWebcam.cameraCalibration.principalPointY)
                                .build();

                        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(configuredWebcam,
                                Pair.create(processorId, aprilTagProcessor));
                        configuredWebcam.setVisionPortalWebcam(aprilTagWebcam);
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Invalid processor id " + processorId);
                }

                break;
            }

            case "WAIT_FOR_WEBCAM_START": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);

                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to wait for the startup of webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                int timeout = actionXPath.getRequiredInt("timeout_ms");
                if (!configuredWebcam.getVisionPortalWebcam().waitForWebcamStart(timeout)) {
                    return false; // no webcam, just give up
                }

                break;
            }

            case "STOP_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);

                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to close webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                configuredWebcam.getVisionPortalWebcam().finalShutdown();
                configuredWebcam.setVisionPortalWebcam(null);
                openWebcam = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;
                RobotLogCommon.d(TAG, "Stopped webcam " + webcamIdString);

                break;
            }

            case "STOP_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to stop streaming webcam " + webcamId + " but it is not open");

                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().stopStreaming();
                RobotLogCommon.d(TAG, "Stopped streaming webcam " + webcamIdString);
                break;
            }

            case "RESUME_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to resume streaming webcam " + webcamId + " but it is not open");

                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().resumeStreaming();
                RobotLogCommon.d(TAG, "Resumed streaming webcam " + webcamIdString);
                break;
            }

            case "ENABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to enable processor on webcam " + webcamId + " but it is not open");

                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().enableProcessor();
                RobotLogCommon.d(TAG, "Enabled processor on webcam " + webcamIdString);
                break;
            }

            case "DISABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to disable processor on webcam " + webcamId + " but it is not open");

                Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam().disableProcessor();
                RobotLogCommon.d(TAG, "Disabled processor on webcam " + webcamIdString);
                break;
            }

            // For testing, get a frame from a webcam managed by the VisionPortal
            // API and write it out to a file. Assume that the webcam has already
            // been started.
            case "TAKE_PICTURE_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to take picture on webcam " + webcamId + " but it is not open");

                RawFrameWebcam rawFrameWebcam = (RawFrameWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                Pair<Mat, Date> image = rawFrameWebcam.getImage();
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
                SpikeWindowMapping opModeSpikeWindowMapping =
                        spikeWindowMappingXML.collectSpikeWindowMapping(pOpMode);

                if (opModeSpikeWindowMapping == null)
                    throw new AutonomousRobotException(TAG, "Element 'FIND_TEAM_PROP' not found under OpMode " + pOpMode);

                String webcamIdString = opModeSpikeWindowMapping.imageParameters.image_source.toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to find the team prop using webcam " + webcamId + " but it is not open");

                RawFrameWebcam rawFrameWebcam = (RawFrameWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();

                // Get the recognition path from the XML file.
                RobotConstantsCenterStage.TeamPropRecognitionPath teamPropRecognitionPath =
                        opModeSpikeWindowMapping.recognitionPath;
                RobotLogCommon.d(TAG, "Recognition path " + teamPropRecognitionPath);

                // Perform image recognition.
                TeamPropReturn teamPropReturn =
                        teamPropRecognition.recognizeTeamProp(rawFrameWebcam, teamPropRecognitionPath, teamPropParameters, opModeSpikeWindowMapping);

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

            //**TODO TEMP for testing 10/20/2023
            case "TEST_MULTIPORTAL_SAMPLE": {
                MultiPortalAuto multiPortalAuto = new MultiPortalAuto(linearOpMode,
                        Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.REAR_WEBCAM)).getWebcamName(), // assume this is "Webcam 1"
                        Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM)).getWebcamName()); // assume this is "Webcam 2"
                multiPortalAuto.runOpMode();
                break;
            }

            // For testing: look for all AprilTags in a loop for 5 seconds.
            case "FIND_ALL_APRIL_TAGS": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to find AprilTags using webcam " + webcamId + " but it is not open");

                AprilTagWebcam aprilTagWebcam = (AprilTagWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();

                ElapsedTime aprilTagTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                aprilTagTimer.reset();
                List<AprilTagDetection> currentDetections;
                boolean aprilTagDetected;
                while (linearOpMode.opModeIsActive() && aprilTagTimer.time() < 10000) {
                    aprilTagDetected = false;
                    currentDetections = aprilTagWebcam.getAprilTagData(500);
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            aprilTagDetected = true;
                            break; // don't look any further.
                        }
                    }

                    if (!aprilTagDetected) {
                        linearOpMode.telemetry.addLine("No AprilTags found within 500ms");
                        linearOpMode.telemetry.update();
                        RobotLogCommon.d(TAG, "No AprilTags found within 500ms");
                    } else
                        telemetryAprilTag(currentDetections);

                    sleep(250); // be careful - small sleep values flood the log
                }

                break;
            }

            // Look for a specific AprilTag
            case "FIND_APRIL_TAG": {
                String tagIdString = actionXPath.getRequiredText("tag_id").toUpperCase();
                RobotConstantsCenterStage.AprilTagId targetTagId = RobotConstantsCenterStage.AprilTagId.valueOf(tagIdString);
                findBackdropAprilTag(targetTagId, actionXPath);
                break;
            }

            // Locate a specific AprilTag and drive the robot into position
            // in front of it with the method we used in PowerPlay.
            case "DRIVE_TO_APRIL_TAG": {
                // This check is crucial here because the code in this case block
                // moves the elevator asynchronously.
                if (asyncMoveElevator != null) // Prevent double initialization
                    throw new AutonomousRobotException(TAG, "asyncMoveElevator is already in progress");

                // Movement must start at the SAFE level.
                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE)
                    throw new AutonomousRobotException(TAG, "Move to the AUTONOMOUS level may not start at elevator " + currentElevatorLevel);

                deskew(); // face the AprilTag(s), i.e. cut the yaw to 0

                String tagIdString = actionXPath.getRequiredText("tag_id").toUpperCase();
                RobotConstantsCenterStage.AprilTagId targetTagId = RobotConstantsCenterStage.AprilTagId.valueOf(tagIdString);

                Pair<RobotConstantsCenterStage.AprilTagId, AprilTagDetection> detectionData = findBackdropAprilTag(targetTagId, actionXPath);
                if (detectionData.second == null) {
                    return false; // no sure path to the backdrop
                }

                // If the backstop AprilTag that was found is not our target tag
                // then infer the position of the target tag.
                double aprilTagDistance;
                double aprilTagAngle;
                if (detectionData.first != targetTagId) {
                    RobotLogCommon.d(TAG, "Did not detect the target AprilTag " + targetTagId);
                    RobotLogCommon.d(TAG, "Inferring its position from tag " + detectionData.first);
                    Pair<Double, Double> inferredPosition = AprilTagUtils.inferAprilTag(targetTagId, detectionData.first,
                            detectionData.second.ftcPose.range, detectionData.second.ftcPose.bearing);
                    aprilTagDistance = inferredPosition.first;
                    aprilTagAngle = inferredPosition.second;
                    RobotLogCommon.d(TAG, "Inferred distance " + aprilTagDistance + ", angle " + aprilTagAngle);
                } else {
                    aprilTagDistance = detectionData.second.ftcPose.range;
                    aprilTagAngle = detectionData.second.ftcPose.bearing;
                    RobotLogCommon.d(TAG, "Using target AprilTag distance " + aprilTagDistance + ", angle " + aprilTagAngle);
                }

                // The deskew above should have taken care of the yaw but let's see
                // what the AprilTag detector thinks it is.
                RobotLogCommon.d(TAG, "Yaw as reported by the AprilTag detector " + detectionData.second.ftcPose.yaw);

                double desiredDistanceFromTag = actionXPath.getRequiredDouble("desired_distance_from_tag");
                String directionString = actionXPath.getRequiredText("direction").toUpperCase();
                DriveTrainConstants.Direction direction =
                        DriveTrainConstants.Direction.valueOf(directionString);

                RobotLogCommon.d(TAG, "Driving to AprilTag with id " + targetTagId);
                RobotLogCommon.d(TAG, "Stop at " + desiredDistanceFromTag + " from the tag");
                RobotLogCommon.d(TAG, "Direction of travel " + direction);

                // Unlike the RobotAutoDriveToAprilTagOmni sample, which tracks the
                // AprilTag in relation to the camera, we need the angle and distance
                // from the center of the robot, particularly if the camera is not
                // centered on the robot.
                double angleFromRobotCenterToAprilTag =
                        CameraToCenterCorrections.getCorrectedAngle(backdropParameters.distanceCameraLensToRobotCenter,
                                backdropParameters.offsetCameraLensFromRobotCenter, aprilTagDistance, aprilTagAngle);

                double distanceFromRobotCenterToAprilTag =
                        CameraToCenterCorrections.getCorrectedDistance(backdropParameters.distanceCameraLensToRobotCenter,
                                backdropParameters.offsetCameraLensFromRobotCenter, aprilTagDistance, aprilTagAngle);

                double distanceToMove;
                if (Math.abs(angleFromRobotCenterToAprilTag) >= 3.0) {
                    // Strafe to place the center of the robot opposite the AprilTag.
                    double sinTheta = Math.sin(Math.toRadians(Math.abs(angleFromRobotCenterToAprilTag)));
                    double distanceToStrafe = sinTheta * distanceFromRobotCenterToAprilTag;
                    double strafeVelocity = shortDistanceVelocity(distanceToStrafe);

                    // Set the direction to strafe. A positive angle indicates that the
                    // tag is to the left of the center of the robot (clockwise). Take
                    // into account the robot's direction of travel.
                    double directionFactor = (direction == DriveTrainConstants.Direction.FORWARD) ? 1.0 : -1.0;
                    double strafeDirection = (angleFromRobotCenterToAprilTag > 0 ? 90.0 : -90.0) * directionFactor;

                    // Add in strafe percentage adjustment.
                    if (backdropParameters.strafeAdjustmentPercent != 0.0) {
                        distanceToStrafe += (distanceToStrafe * backdropParameters.strafeAdjustmentPercent);
                        RobotLogCommon.d(TAG, "Adjusting distance to strafe by " + backdropParameters.strafeAdjustmentPercent);
                    }

                    int targetClicks = (int) (distanceToStrafe * robot.driveTrain.getClicksPerInch());
                    driveTrainMotion.straight(targetClicks, strafeDirection, strafeVelocity, 0, desiredHeading);
                    RobotLogCommon.d(TAG, "Strafe towards the AprilTag " + distanceToStrafe + " inches at " + strafeDirection + " degrees");

                    // Calculate the distance to move towards the backstop based on our triangle.
                    // distanceFromRobotCenterToAprilTag (hypotenuse) squared = distanceToStrafe squared + adjacent squared.
                    double adjacentSquared = Math.pow(distanceFromRobotCenterToAprilTag, 2) - Math.pow(distanceToStrafe, 2);
                    double adjacent = Math.sqrt(adjacentSquared); // center of robot to AprilTag
                    distanceToMove = adjacent - (backdropParameters.distanceCameraLensToRobotCenter + desiredDistanceFromTag);
                    RobotLogCommon.d(TAG, "Adjusted pythagorean distance to move towards the backdrop " + distanceToMove);
                } else {
                    distanceToMove = distanceFromRobotCenterToAprilTag - (backdropParameters.distanceCameraLensToRobotCenter + desiredDistanceFromTag);
                    RobotLogCommon.d(TAG, "Calculated distance to move towards the backdrop " + distanceToMove);
                }

                // Start the elevator moving up to the AUTONOMOUS level asynchronously.
                Callable<Void> callableMove = async_move_elevator_and_winch(Elevator.ElevatorLevel.AUTONOMOUS);
                asyncMoveElevator = Threading.launchAsync(callableMove);
                RobotLogCommon.d(TAG, "Start asynchronous elevator/winch movement to the AUTONOMOUS level");

                // Move the robot towards the backstop. Take into account the robot's direction of travel.
                // Add in distance percentage adjustment.
                if (backdropParameters.distanceAdjustmentPercent != 0.0) {
                    distanceToMove += (distanceToMove * backdropParameters.distanceAdjustmentPercent);
                    RobotLogCommon.d(TAG, "Adjusting distance to move by " + backdropParameters.distanceAdjustmentPercent);
                }

                double moveAngle = (direction == DriveTrainConstants.Direction.FORWARD) ? 0.0 : -180.0;
                double straightLineVelocity = shortDistanceVelocity(distanceToMove);
                if (Math.abs(distanceToMove) >= 1.0) {
                    RobotLogCommon.d(TAG, "Move robot towards the AprilTag " + distanceToMove + " inches");
                    int targetClicks = (int) (Math.abs(distanceToMove) * robot.driveTrain.getClicksPerInch());
                    driveTrainMotion.straight(targetClicks, moveAngle, straightLineVelocity, 0, desiredHeading);
                }

                // Wait for the elevator/winch movement to complete.
                Threading.getFutureCompletion(asyncMoveElevator);
                RobotLogCommon.d(TAG, "Async move elevator/winch: complete");
                asyncMoveElevator = null;
                break;
            }

            // Locate a specific AprilTag and drive the robot into position
            // in front of it with the navigation method from the FTC sample
            // RobotAutoDriveToAprilTagOmni.
            case "NAVIGATE_TO_APRIL_TAG": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsCenterStage.InternalWebcamId webcamId =
                        RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
                if (openWebcam != webcamId)
                    throw new AutonomousRobotException(TAG, "Attempt to navigate to AprilTags using webcam " + webcamId + " but it is not open");

                // If the internal id of the webcam in the current AprilTagNavigation object
                // does not match the id just specified then recreate the AprilTagNavigation object.
                if (aprilTagNavigation == null || aprilTagNavigation.getInternalWebcamId() !=
                        webcamId) {
                    RobotLogCommon.d(TAG, "Switching AprilTag navigation to " + webcamIdString);
                    AprilTagWebcam aprilTagWebcam = (AprilTagWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
                    aprilTagNavigation = new AprilTagNavigation(alliance, linearOpMode, robot, aprilTagWebcam);
                }

                int desiredTagId = actionXPath.getRequiredInt("tag_id");
                double desiredDistanceFromTag = actionXPath.getRequiredDouble("desired_distance_from_tag");

                String directionString = actionXPath.getRequiredText("direction").toUpperCase();
                DriveTrainConstants.Direction direction =
                        DriveTrainConstants.Direction.valueOf(directionString);
                RobotLogCommon.d(TAG, "Navigating to AprilTag with id " + desiredTagId);
                RobotLogCommon.d(TAG, "Stop at " + desiredDistanceFromTag + " from the tag");
                RobotLogCommon.d(TAG, "Direction of travel " + directionString);
                if (!aprilTagNavigation.navigateToAprilTag(desiredTagId, desiredDistanceFromTag, direction)) {
                    RobotLogCommon.d(TAG, "Navigation to AprilTag was not successful");
                    return false;
                }

                deskew(); // make sure the robot is aligned with the desired heading
                break;
            }

            case "PIXEL_STOPPER": {
                String positionString = actionXPath.getRequiredText("position").toUpperCase();
                PixelStopperServo.PixelServoState position =
                        PixelStopperServo.PixelServoState.valueOf(positionString);
                if (position == PixelStopperServo.PixelServoState.HOLD) {
                    robot.pixelStopperServo.hold(); // hold for intake of pixels from the front
                    pixelServoState = PixelStopperServo.PixelServoState.HOLD;
                } else {
                    robot.pixelStopperServo.release(); // release for outtake of pixels out the back
                    pixelServoState = PixelStopperServo.PixelServoState.RELEASE;
                }
                break;
            }

            case "DELIVER_PIXEL_TO_SPIKE": {
                // The position of the pixel stopper does not matter.
                robot.intakeMotion.resetAndMoveSingleMotor(robot.intakeMotor.deliver_front, 0.5, SingleMotorMotion.MotorAction.MOVE_AND_STOP);
                break;
            }

            // Assumes the robot is in position in front of the target
            // backstop and that the elevator is at the AUTONOOUS
            // level.
            case "DELIVER_PIXEL_TO_BACKSTOP": {
                if (pixelServoState != PixelStopperServo.PixelServoState.RELEASE) {
                    robot.pixelStopperServo.release();
                    pixelServoState = PixelStopperServo.PixelServoState.RELEASE;
                }

                // Note that the click count is negated for delivery to the rear.
                robot.intakeMotion.resetAndMoveSingleMotor(-robot.intakeMotor.deliver_back, robot.intakeMotor.velocity, SingleMotorMotion.MotorAction.MOVE_AND_STOP);
                break;
            }

            // Move the elevator to an absolute position and, for all positions
            // other than "ground", hold that position.
            case "MOVE_ELEVATOR": {
                if (asyncMoveElevator != null)
                    throw new AutonomousRobotException(TAG, "asyncMoveElevator is in progress");

                String position = actionXPath.getRequiredText("position").toUpperCase();
                Elevator.ElevatorLevel targetLevel = Elevator.ElevatorLevel.valueOf(position);
                move_elevator_and_winch_to_selected_level(targetLevel);
                break;
            }

            case "ASYNC_MOVE_ELEVATOR": {
                String operation = actionXPath.getRequiredTextInRange("operation", actionXPath.validRange("start", "wait"));
                switch (operation) {
                    case "start": {
                        if (asyncMoveElevator != null) // Prevent double initialization
                            throw new AutonomousRobotException(TAG, "asyncMoveElevator is already in progress");

                        RobotLogCommon.d(TAG, "ASYNC_MOVE_ELEVATOR start");
                        String position = actionXPath.getRequiredText("position").toUpperCase();
                        Elevator.ElevatorLevel targetLevel = Elevator.ElevatorLevel.valueOf(position);
                        Callable<Void> callableMove = async_move_elevator_and_winch(targetLevel);
                        asyncMoveElevator = Threading.launchAsync(callableMove);
                        break;
                    }

                    // Wait for the elevator move to complete.
                    case "wait": {
                        if (asyncMoveElevator == null)
                            throw new AutonomousRobotException(TAG, "In wait: asyncMoveElevator has not been initalized");

                        RobotLogCommon.d(TAG, "ASYNC_MOVE_ELEVATOR wait");
                        Threading.getFutureCompletion(asyncMoveElevator);
                        RobotLogCommon.d(TAG, "ASYNC_MOVE_ELEVATOR wait complete");
                        asyncMoveElevator = null;
                        break;
                    }

                    default:
                        throw new AutonomousRobotException(TAG, "Invalid asynchronous move elevator operation: " + operation);
                }
                break;
            }

            case "SLEEP": { // I want sleep :)
                int sleepMs = actionXPath.getRequiredInt("ms");
                sleepInLoop(sleepMs);
                break;
            }

            // Final strafe: for eah of the 4 OpModes there are
            // three Team Prop locations with two options each
            // for the robot's final strafe (to park safely away
            // from the alliance partner. The ending position is
            // described from the point of view of a person
            // standing in front of the Backdrop. Since the robot
            // is facing backwards the actual strafe will be the
            // inverse.
            case "AUTO_ENDING_POSITION": {
                // Use the current competition OpMode to look up the
                // the desired final position of the robot with respect
                // to the backdrop.
                if (pOpMode.getOpModeType() != RobotConstantsCenterStage.OpMode.OpModeType.COMPETITION)
                    throw new AutonomousRobotException(TAG, "AUTO_ENDING_POSITION only applies to competition OpModes");
                switch (autoEndingPositions.get(pOpMode)) {
                    case LEFT: {
                        // Follow the XPath to the STRAFE child of AUTO_ENDING_POSITION/left
                        // and execute it.
                        Pair<RobotXMLElement, Double> strafeToLeftPosition = RobotActionXMLCenterStage.getFinalPositionElement(pAction, "LEFT");
                        RobotLogCommon.d(TAG, "Ending strafe LEFT");
                        straight_by(new XPathAccess(strafeToLeftPosition.first), () -> strafeToLeftPosition.second).call();
                        break;
                    }
                    case RIGHT: {
                        // Follow the XPath to the STRAFE child of AUTO_ENDING_POSITION/right
                        // and execute it.
                        Pair<RobotXMLElement, Double> strafeToRightPosition = RobotActionXMLCenterStage.getFinalPositionElement(pAction, "RIGHT");
                        RobotLogCommon.d(TAG, "Ending strafe RIGHT");
                        straight_by(new XPathAccess(strafeToRightPosition.first), () -> strafeToRightPosition.second).call();
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Unrecognized autonomous ending position");
                }
                break;
            }

            // In testing this gives us a way to short-circuit a set
            //  of actions without commenting out any XML.
            case "STOP": {
                sleep(500);
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
                    heading = robot.imuDirect.getIMUHeading();
                    pitch = robot.imuDirect.getIMUPitch();
                    roll = robot.imuDirect.getIMURoll();

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

        int numSleeps = pMilliseconds / 100;
        int sleepRemainder = pMilliseconds % 100;
        for (int i = 0; i < numSleeps; i++) {
            if (!linearOpMode.opModeIsActive())
                return;
            linearOpMode.sleep(100);
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
                RobotLogCommon.d(TAG, "async_straight_by start");
                Callable<Void> callableDriveToPosition =
                        straight_by(pActionXPath, pAngle);
                asyncStraight = Threading.launchAsync(callableDriveToPosition);
                break;
            }

            // Wait for the straight run to complete.
            case "wait": {
                if (asyncStraight == null)
                    throw new AutonomousRobotException(TAG, "In wait: asyncStraight has not been initalized");

                RobotLogCommon.d(TAG, "async_straight_by wait");
                Threading.getFutureCompletion(asyncStraight);
                asyncStraight = null;
                RobotLogCommon.d(TAG, "async_straight_by wait complete");
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
                RobotLogCommon.d(TAG, "ASYNC_TURN start");
                break;
            }

            // Wait for the turn to complete.
            case "wait": {
                if (asyncTurn == null)
                    throw new AutonomousRobotException(TAG, "In wait: asyncTurn has not been initalized");

                RobotLogCommon.d(TAG, "ASYNC_TURN wait");
                desiredHeading = Threading.getFutureCompletion(asyncTurn);
                asyncTurn = null;
                break;
            }

            default:
                throw new AutonomousRobotException(TAG, "Invalid asynchronous turn operation: " + operation);
        }
    }

    // Return a Callable that can be used to turn the robot directly or can be launched
    // to turn the robot asynchronously.
    private Callable<Double> turn(XPathAccess pActionXPath) throws XPathExpressionException {
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
        double currentHeading = robot.imuDirect.getIMUHeading();
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

        final double finalRampDownAtAngleRemaining = rampDownAtAngleRemaining; // required for lambda
        final double finalAngle = angle;
        return () -> driveTrainMotion.turn(desiredHeading, robot.imuDirect.getIMUHeading(), finalAngle, power, finalRampDownAtAngleRemaining, turnNormalization);
    }

    // Straighten out the robot by turning to the desired heading.
    // Make a normalized turn at minimum power.
    private void deskew() throws IOException, InterruptedException, TimeoutException {
        if (asyncStraight != null)
            throw new AutonomousRobotException(TAG, "Deskew not allowed while asyncStraight is in progress");

        if (asyncTurn != null)
            throw new AutonomousRobotException(TAG, "Deskew not allowed while asyncTurn is in progress");

        double currentHeading = robot.imuDirect.getIMUHeading();
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

    @SuppressLint("DefaultLocale")
    private Pair<RobotConstantsCenterStage.AprilTagId, AprilTagDetection> findBackdropAprilTag(RobotConstantsCenterStage.AprilTagId pTargetTagId, XPathAccess pActionXPath) throws XPathExpressionException {
        String webcamIdString = pActionXPath.getRequiredText("internal_webcam_id").toUpperCase();
        RobotConstantsCenterStage.InternalWebcamId webcamId =
                RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
        if (openWebcam != webcamId)
            throw new AutonomousRobotException(TAG, "Attempt to find AprilTags using webcam " + webcamId + " but it is not open");

        int timeout = pActionXPath.getRequiredInt("timeout_ms");

        AprilTagWebcam aprilTagWebcam = (AprilTagWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();
        List<AprilTagDetection> currentDetections = aprilTagWebcam.getAprilTagData(timeout);
        AprilTagDetection targetDetection = null;
        AprilTagDetection backupDetection = null;
        double smallestBackupAngle = 360.0; // impossibly high

        // Step through the list of detected tags and look for a matching tag.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == pTargetTagId.getNumericId()) {
                    targetDetection = detection;
                    break; // don't look any further.
                } else {
                    if (Math.abs(detection.ftcPose.bearing) < smallestBackupAngle) {
                        smallestBackupAngle = detection.ftcPose.bearing;
                        backupDetection = detection;
                    }
                }
            }
        }

        // If we have found the target AprilTag return it now.
        if (targetDetection != null) {
            String targetTagString = "Found target AprilTag " + String.format("Id %d (%s)", targetDetection.id, targetDetection.metadata.name);
            String range = "Range " + String.format("%5.1f inches", targetDetection.ftcPose.range);
            String bearing = "Bearing " + String.format("%3.0f degrees", targetDetection.ftcPose.bearing);
            String yaw = "Yaw " + String.format("%3.0f degrees", targetDetection.ftcPose.yaw);

            linearOpMode.telemetry.addLine(targetTagString);
            linearOpMode.telemetry.update();

            RobotLogCommon.d(TAG, targetTagString);
            RobotLogCommon.d(TAG, range);
            RobotLogCommon.d(TAG, bearing);
            RobotLogCommon.d(TAG, yaw);
            return Pair.create(pTargetTagId, targetDetection);
        }

        // If we have not found the target target, see if we've found one of
        // the other AprilTags to use as a backup.
        if (backupDetection == null) {
            linearOpMode.telemetry.addLine("No AprilTags found within " + timeout + "ms");
            linearOpMode.telemetry.update();
            RobotLogCommon.d(TAG, "No AprilTags found within " + timeout + "ms");
            return Pair.create(pTargetTagId, null);
        }

        // Found a backup detection.
        RobotConstantsCenterStage.AprilTagId backupTagId = getEnumValue(backupDetection.id);
        String backupTagString = "Found backup AprilTag " + String.format("Id %d (%s)", backupDetection.id, backupDetection.metadata.name);
        String range = "Range " + String.format("%5.1f inches", backupDetection.ftcPose.range);
        String bearing = "Bearing " + String.format("%3.0f degrees", backupDetection.ftcPose.bearing);
        String yaw = "Yaw " + String.format("%3.0f degrees", backupDetection.ftcPose.yaw);

        linearOpMode.telemetry.addLine(backupTagString);
        linearOpMode.telemetry.update();

        RobotLogCommon.d(TAG, backupTagString);
        RobotLogCommon.d(TAG, range);
        RobotLogCommon.d(TAG, bearing);
        RobotLogCommon.d(TAG, yaw);
        return Pair.create(backupTagId, backupDetection);
    }

    private Callable<Void> async_move_elevator_and_winch(Elevator.ElevatorLevel pElevatorTargetLevel) {
        return () -> {
            move_elevator_and_winch_to_selected_level(pElevatorTargetLevel);
            return null;
        };
    }

    // Based on methods from CenterStageTeleOp.java as of 12/12/2023.
    // This method assumes that the caller's has made sure that an
    // asynchronous movement of the elevator is not already in progress.
    private void move_elevator_and_winch_to_selected_level(Elevator.ElevatorLevel pSelectedLevel) throws IOException, InterruptedException, TimeoutException {
        CompletableFuture<Elevator.ElevatorLevel> localAsyncElevator = null;
        CompletableFuture<Winch.WinchLevel> localAsyncWinch = null;

        switch (pSelectedLevel) {
            case GROUND: {
                if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND)
                    return; // already there

                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE)
                    throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator to GROUND from " + currentElevatorLevel);

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to GROUND");
                robot.elevatorMotion.moveDualMotors(robot.elevator.ground, robot.elevator.getVelocity(), DualMotorMotion.DualMotorAction.MOVE_AND_STOP);
                currentElevatorLevel = Elevator.ElevatorLevel.GROUND;
                break;
            }
            case SAFE: {
                if (currentElevatorLevel == Elevator.ElevatorLevel.SAFE)
                    return; // already there

                if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND) { // upward movement?
                    RobotLogCommon.d(TAG, "Moving elevator up from GROUND to SAFE");
                    robot.elevatorMotion.moveDualMotors(robot.elevator.safe, robot.elevator.getVelocity(), DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                    currentElevatorLevel = Elevator.ElevatorLevel.SAFE;
                } else { // downward movement
                    RobotLogCommon.d(TAG, "Moving elevator down to SAFE at velocity " + robot.elevator.velocity_down);
                    localAsyncElevator = async_move_elevator(robot.elevator.safe, robot.elevator.velocity_down, Elevator.ElevatorLevel.SAFE);
                    if (robot.winch != null)
                        localAsyncWinch = async_move_winch(robot.winch.safe, Winch.WinchLevel.SAFE);
                }

                break;
            }
            case AUTONOMOUS: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE)
                    throw new AutonomousRobotException(TAG, "The elevator must be at SAFE before moving to AUTONOMOUS");

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to AUTONOMOUS");
                localAsyncElevator = async_move_elevator(Objects.requireNonNull(robot.elevator).autonomous, robot.elevator.getVelocity(), Elevator.ElevatorLevel.AUTONOMOUS);
                if (robot.winch != null) // the winch is configured in
                    localAsyncWinch = async_move_winch(Objects.requireNonNull(robot.winch).autonomous, Winch.WinchLevel.AUTONOMOUS);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Invalid elevator level " + pSelectedLevel);
        }

        // If either the elevator or both the elevator and winch have been started asynchronously
        // wait for completion here. The winch never moves asynchronously without the elevator.
        if (localAsyncElevator != null) {
            RobotLogCommon.d(TAG, "Wait for asynchonous movement of the elevator to complete");
            currentElevatorLevel = Threading.getFutureCompletion(localAsyncElevator);
            RobotLogCommon.d(TAG, "Asynchonous movement of the elevator is complete");

            if (localAsyncWinch != null) {
                RobotLogCommon.d(TAG, "Wait for asynchonous movement of the winch to complete");
                Winch.WinchLevel currentWinchLevel = Threading.getFutureCompletion(localAsyncWinch);
                RobotLogCommon.d(TAG, "Asynchonous movement of the winch to " + currentWinchLevel + " is complete");
            }
        }
    }

    // This method assumes that the caller's has made sure that an
    // asynchronous movement of the elevator is not already in progress.
    private CompletableFuture<Elevator.ElevatorLevel> async_move_elevator(int pElevatorPosition, double pElevatorVelocity, Elevator.ElevatorLevel pElevatorLevelOnCompletion) {
        Callable<Elevator.ElevatorLevel> callableMoveElevator = () -> {
            robot.elevatorMotion.moveDualMotors(pElevatorPosition, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return pElevatorLevelOnCompletion;
        };

        RobotLogCommon.d(TAG, "Async MOVE_ELEVATOR_AND_WINCH in progress");
        return Threading.launchAsync(callableMoveElevator);
    }

    // This method assumes that the caller's has made sure that an
    // asynchronous movement of the elevator is not already in progress.
    private CompletableFuture<Winch.WinchLevel> async_move_winch(int pWinchPosition, Winch.WinchLevel pWinchLevelOnCompletion) {
        Callable<Winch.WinchLevel> callableMoveWinch = () -> {
            robot.winchMotion.moveSingleMotor(pWinchPosition, robot.winch.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            return pWinchLevelOnCompletion;
        };

        RobotLogCommon.d(TAG, "Async move winch in progress");
        return Threading.launchAsync(callableMoveWinch);
    }

    // Failsafe with the goal of making sure that the elevator is at GROUND.
    private void failsafeElevator() {
        if (robot.elevator == null) {
            RobotLogCommon.d(TAG, "The elevator is not in the current configuration");
            return;
        }

        if (asyncMoveElevator != null) {
            RobotLogCommon.d(TAG, "asyncMoveElevator is active in failsafe; there's nothing we can do");
            return;
        }

        if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND) {
            RobotLogCommon.d(TAG, "The elevator is already at the GROUND level");
            return;
        }

        // Simply move the elevator to GROUND.
        robot.elevatorMotion.moveDualMotors(robot.elevator.ground, Objects.requireNonNull(robot.elevator, "robot.elevatorMotors unexpectedly null").getVelocity(),
                DualMotorMotion.DualMotorAction.MOVE_AND_STOP);

        RobotLogCommon.i(TAG, "Done with failsafe actions");
    }

}

