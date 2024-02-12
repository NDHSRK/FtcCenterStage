package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.xml.TeamPropParameters;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.firstinspires.ftc.teamcode.xml.TeamPropParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.xml.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.PixelCountRendering;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

// This OpMode gives the drive team a way to check and
// modify the grayscale thresholding of the cropped
// webcam frame that contains the Team Prop.
@TeleOp(name = "PixelCountViewer", group = "Configure")
//@Disabled
public class PixelCountViewer extends LinearOpMode {
    private static final String TAG = PixelCountViewer.class.getSimpleName();
    private static final int THRESHOLD_CHANGE = 5;

    private TeamPropParametersXML teamPropParametersXML;
    private TeamPropParameters teamPropParameters;
    private EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowMapping> collectedSpikeWindowMapping;
    private CameraStreamProcessor pixelCountProcessor;
    RobotConstantsCenterStage.OpMode currentOpMode = RobotConstantsCenterStage.OpMode.OPMODE_NPOS;
    private FTCButton opModeBlueA2;
    private FTCButton opModeBlueA4;
    private FTCButton opModeRedF4;
    private FTCButton opModeRedF2;

    //## Structures are in place - here and in TeamPropParametersXML -
    // to support changes to the grayscale median target. But the
    // buttons and associated methods must be created. It's not
    // clear how useful it is to change the median.
    // private FTCButton increaseMedian;
    // private FTCButton decreaseMedian;

    private FTCButton increaseThreshold;
    private FTCButton decreaseThreshold;
    private FTCButton requestImageCapture;
    private PixelCountRendering pixelCountRendering;
    private RobotConstants.Alliance alliance = RobotConstants.Alliance.NONE;
    private VisionParameters.GrayParameters opModeGrayParameters;
    private int currentThresholdLow;
    private int currentMinWhitePixelCount;
    private boolean grayscaleParametersChanged = false;

    // In this OpMode all of the action takes place during init().
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.ii(TAG, "Initializing the PixelCountViewer");

        // Read the parameters for team prop recognition from the xml file.
        teamPropParametersXML = new TeamPropParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
        teamPropParameters = teamPropParametersXML.getTeamPropParameters();

        // Get the camera configuration from RobotConfig.xml.
        FTCRobot robot = new FTCRobot(this, RobotConstants.RunType.TELEOP_VISION_PREVIEW);

        // Start the front webcam with the pixel count window processor.
        if (robot.configuredWebcams == null)
            throw new AutonomousRobotException(TAG, "There are no webcams in the current configuration");

        VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM),
                TAG + " The FRONT_WEBCAM is not configured");

        pixelCountProcessor = new CameraStreamProcessor.Builder().build();
        VisionPortalWebcam pixelCountWebcam = new VisionPortalWebcam(frontWebcamConfiguration,
                RobotConstantsCenterStage.ProcessorIdentifier.CAMERA_STREAM_PREVIEW,
                Pair.create(pixelCountProcessor, true));

        if (!pixelCountWebcam.waitForWebcamStart(2000))
            throw new AutonomousRobotException(TAG, "Pixel count webcam timed out on start");

        frontWebcamConfiguration.setVisionPortalWebcam(pixelCountWebcam);
        RobotLog.ii(TAG, "PixelCountViewer successfully started on the front webcam");

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        try {
            SpikeWindowMappingXML spikeWindowMappingXML = new SpikeWindowMappingXML(robot.startParameters.robotActionFilename);
            collectedSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping();
        } catch (ParserConfigurationException | IOException | SAXException |
                 XPathExpressionException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }

        // Set up the DPAD buttons for starting position selection - clockwise
        // from the audience wall.
        opModeBlueA2 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        opModeBlueA4 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        opModeRedF4 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        opModeRedF2 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);
        increaseThreshold = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        decreaseThreshold = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);
        requestImageCapture = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayerOne();

            updateTelemetry();
        }

        if (opModeIsActive()) {
            // If there have been any changes to the grayscale values
            // write them out to TeamPropParameters.xml now.
            if (grayscaleParametersChanged) {
                teamPropParametersXML.writeTeamPropParametersFile();
                RobotLog.ii(TAG, "Writing TeamPropParameters.xml");
                telemetry.addLine("Writing TeamPropParameters.xml");
                telemetry.update();
                sleep(1500);
            }
            telemetry.addLine("Ending the PixelCountViewer");
            telemetry.update();
        }
    }

    private void updateButtons() {
        opModeBlueA2.update();
        opModeBlueA4.update();
        opModeRedF4.update();
        opModeRedF2.update();
        increaseThreshold.update();
        decreaseThreshold.update();
        requestImageCapture.update();
    }

    private void updatePlayerOne() {
        updateOpModeBlueA2();
        updateOpModeBlueA4();
        updateOpModeRedF4();
        updateOpModeRedF2();
        updateIncreaseThreshold();
        updateDecreaseThreshold();
        updateRequestImageCapture();
    }

    private void updateOpModeBlueA2() {
        currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A2;
        setPixelCountRendering(RobotConstantsCenterStage.OpMode.BLUE_A2, opModeBlueA2);
    }

    private void updateOpModeBlueA4() {
        currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A4;
        setPixelCountRendering(RobotConstantsCenterStage.OpMode.BLUE_A4, opModeBlueA4);
    }

    private void updateOpModeRedF4() {
        currentOpMode = RobotConstantsCenterStage.OpMode.RED_F4;
        setPixelCountRendering(RobotConstantsCenterStage.OpMode.RED_F4, opModeRedF4);
    }

    private void updateOpModeRedF2() {
        currentOpMode = RobotConstantsCenterStage.OpMode.RED_F2;
        setPixelCountRendering(RobotConstantsCenterStage.OpMode.RED_F2, opModeRedF2);
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateIncreaseThreshold() {
        if (increaseThreshold.is(FTCButton.State.TAP)) {
            if (pixelCountRendering == null || currentThresholdLow == 255)
                return; // no OpMode has been selected; can't go above maximum

            currentThresholdLow += THRESHOLD_CHANGE;

            VisionParameters.GrayParameters updatedVisionParameters = new VisionParameters.GrayParameters(opModeGrayParameters.median_target, currentThresholdLow);
            teamPropParametersXML.setPixelCountGrayParameters(alliance, updatedVisionParameters);
            grayscaleParametersChanged = true;
            pixelCountRendering.setGrayscaleThresholdParameters(updatedVisionParameters);
        }
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateDecreaseThreshold() {
        if (decreaseThreshold.is(FTCButton.State.TAP)) {
            if (pixelCountRendering == null || currentThresholdLow == 0)
                return; // no OpMode has been selected; can't go below minimum

            currentThresholdLow -= THRESHOLD_CHANGE;

            VisionParameters.GrayParameters updatedVisionParameters = new VisionParameters.GrayParameters(opModeGrayParameters.median_target, currentThresholdLow);
            teamPropParametersXML.setPixelCountGrayParameters(alliance, updatedVisionParameters);
            grayscaleParametersChanged = true;
            pixelCountRendering.setGrayscaleThresholdParameters(updatedVisionParameters);
        }
    }

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateRequestImageCapture() {
        if (requestImageCapture.is(FTCButton.State.TAP)) {
            if (pixelCountRendering != null)
                pixelCountRendering.requestImageCapture();
        }
    }

    private void setPixelCountRendering(RobotConstantsCenterStage.OpMode pOpMode, FTCButton pOpModeButton) {
        if (pOpModeButton.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "Button " + pOpModeButton.getButtonValue() + " for " + pOpMode + " tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping spikeWindows = collectedSpikeWindowMapping.get(pOpMode);
            if (spikeWindows == null)
                return; // ignore the button click

            VisionParameters.GrayParameters allianceGrayParameters;
            int allianceMinWhitePixelCount;
            if (pOpMode == RobotConstantsCenterStage.OpMode.BLUE_A2 ||
                    pOpMode == RobotConstantsCenterStage.OpMode.BLUE_A4) {
                alliance = RobotConstants.Alliance.BLUE;
                allianceGrayParameters = teamPropParameters.colorChannelPixelCountParameters.blueGrayParameters;
                allianceMinWhitePixelCount = teamPropParameters.colorChannelPixelCountParameters.blueMinWhitePixelCount;
            } else {
                alliance = RobotConstants.Alliance.RED;
                allianceGrayParameters = teamPropParameters.colorChannelPixelCountParameters.redGrayParameters;
                allianceMinWhitePixelCount = teamPropParameters.colorChannelPixelCountParameters.redMinWhitePixelCount;
            }

            opModeGrayParameters = allianceGrayParameters;
            currentThresholdLow = opModeGrayParameters.threshold_low;
            currentMinWhitePixelCount = allianceMinWhitePixelCount;
            pixelCountRendering = new PixelCountRendering(this, pOpMode, alliance, allianceGrayParameters, spikeWindows);
            pixelCountProcessor.setCameraStreamRendering(pixelCountRendering);
            RobotLog.dd(TAG, "Set pixel count rendering for " + pOpMode);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("All pixel count viewing takes place in init");
        telemetry.addLine("Current OpMode " + currentOpMode);
        telemetry.addLine("Select an OpMode");
        telemetry.addLine(" A for BLUE_A2, X for BLUE_A4");
        telemetry.addLine(" Y for RED_F4, B for RED_F2");

        // Show Team Prop locations and pixel counts.
        if (pixelCountRendering != null) {
            Pair<String, String> teamPropResults = pixelCountRendering.getTeamPropResults();
            if (teamPropResults != null) {
                telemetry.addLine("Current minimum pixel count " + currentMinWhitePixelCount);
                telemetry.addLine(teamPropResults.first); // left spike mark in view
                telemetry.addLine(teamPropResults.second); // right spike mark in view
                telemetry.addLine("Current low threshold " + currentThresholdLow);
                telemetry.addLine("Change the threshold");
                telemetry.addLine(" DPAD UP to increase for less white");
                telemetry.addLine(" DPAD DOWN to decrease for more white");
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                telemetry.addData(">", "Touch play to SAVE changes and END the OpMode");
            }
        }

        telemetry.update();
    }

}