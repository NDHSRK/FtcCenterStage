package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.xml.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.SpikeWindowRendering;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

// This OpMode gives the drive team a way to check the
// alignment of the front camera to make sure that the
// Team Prop is in view.
@TeleOp(name = "SpikeWindowViewer", group = "Configure")
//@Disabled
public class SpikeWindowViewer extends LinearOpMode {
    private static final String TAG = SpikeWindowViewer.class.getSimpleName();

    private CameraStreamProcessor spikeWindowProcessor;
    private EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowMapping> collectedSpikeWindowMapping;
    RobotConstantsCenterStage.OpMode currentOpMode = RobotConstantsCenterStage.OpMode.OPMODE_NPOS;
    private FTCButton opModeBlueA2;
    private FTCButton opModeBlueA4;
    private FTCButton opModeRedF4;
    private FTCButton opModeRedF2;

    // In this OpMode all of the action takes place during init().
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.ii(TAG, "Initializing the SpikeWindowViewer");

        // Get the camera configuration from RobotConfig.xml.
        FTCRobot robot = new FTCRobot(this, RobotConstants.RunType.TELEOP_VISION_PREVIEW);

        // Start the front webcam with the spike window processor.
        if (robot.configuredWebcams == null)
            throw new AutonomousRobotException(TAG, "There are no webcams in the current configuration");

        VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM),
                        TAG + " The FRONT_WEBCAM is not configured");

        spikeWindowProcessor = new CameraStreamProcessor.Builder().build();
        VisionPortalWebcam spikeWindowWebcam = new VisionPortalWebcam(frontWebcamConfiguration,
                RobotConstantsCenterStage.ProcessorIdentifier.CAMERA_STREAM_PREVIEW,
                Pair.create(spikeWindowProcessor, true));

        if (!spikeWindowWebcam.waitForWebcamStart(2000))
            throw new AutonomousRobotException(TAG, "Spike window webcam timed out on start");

        frontWebcamConfiguration.setVisionPortalWebcam(spikeWindowWebcam);
        RobotLog.ii(TAG, "SpikeWindowViewer successfully started on the front webcam");

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

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayerOne();

            updateTelemetry();
        }

        telemetry.addLine("Ending the SpikeWindowViewer");
        telemetry.update();
    }

    private void updateButtons() {
        opModeBlueA2.update();
        opModeBlueA4.update();
        opModeRedF4.update();
        opModeRedF2.update();
    }

    private void updatePlayerOne() {
        updateOpModeBlueA2();
        updateOpModeBlueA4();
        updateOpModeRedF4();
        updateOpModeRedF2();
    }

    private void updateOpModeBlueA2() {
        currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A2;
        setSpikeWindowRendering(RobotConstantsCenterStage.OpMode.BLUE_A2, opModeBlueA2);
    }

    private void updateOpModeBlueA4() {
        currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A4;
        setSpikeWindowRendering(RobotConstantsCenterStage.OpMode.BLUE_A4, opModeBlueA4);
    }

    private void updateOpModeRedF4() {
        currentOpMode = RobotConstantsCenterStage.OpMode.RED_F4;
        setSpikeWindowRendering(RobotConstantsCenterStage.OpMode.RED_F4, opModeRedF4);
    }

    private void updateOpModeRedF2() {
        currentOpMode = RobotConstantsCenterStage.OpMode.RED_F2;
        setSpikeWindowRendering(RobotConstantsCenterStage.OpMode.RED_F2, opModeRedF2);
    }

    private void setSpikeWindowRendering(RobotConstantsCenterStage.OpMode pOpMode, FTCButton pOpModeButton) {
        if (pOpModeButton.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "Button " + pOpModeButton.getButtonValue() + " for " + pOpMode + " tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping spikeWindows = collectedSpikeWindowMapping.get(pOpMode);
            if (spikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping on the Driver Station
            // camera stream.
            spikeWindowProcessor.setCameraStreamRendering(new SpikeWindowRendering(spikeWindows));
            RobotLog.dd(TAG, "Set spike window mapping for " + pOpMode);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("All spike window viewing takes place in init");
        telemetry.addLine("Spike windows for " + currentOpMode);
        telemetry.addLine("Select an OpMode");
        telemetry.addLine(" A for BLUE_A2, X for BLUE_A4");
        telemetry.addLine(" Y for RED_F4, B for RED_F2");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch play to *END* the OpMode");
        telemetry.update();
    }

}