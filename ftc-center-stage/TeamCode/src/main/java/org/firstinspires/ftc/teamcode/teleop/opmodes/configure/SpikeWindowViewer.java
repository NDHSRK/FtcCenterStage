package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.auto.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.SpikeWindowProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.SpikeWindowWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

// This OpMode gives the drive team a way to check the
// alignment of the front camera to make sure that the
// Team Prop is in view.
@TeleOp(name = "SpikeWindowViewer", group = "Configure")
//@Disabled
public class SpikeWindowViewer extends LinearOpMode {
    private static final String TAG = SpikeWindowViewer.class.getSimpleName();

    private SpikeWindowProcessor spikeWindowProcessor;
    private EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowMapping> collectedSpikeWindowMapping;
    private FTCButton startPositionF4Button;
    private FTCButton startPositionF2Button;
    private FTCButton startPositionA2Button;
    private FTCButton startPositionA4Button;

    // In this OpMode all of the action takes place during init().
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.ii(TAG, "Initializing the SpikeWindowViewer");

        // Get the camera configuration from RobotConfig.xml.
        FTCRobot robot = new FTCRobot(this, RobotConstants.RunType.TELEOP_VISION_PREVIEW);

        // Start the front webcam with the spike window processor.
        if (robot.configuredWebcams == null || robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM) == null)
            throw new AutonomousRobotException(TAG, "Front camera is not in the current configuration");

        VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);

        spikeWindowProcessor = new SpikeWindowProcessor.Builder().build();
        SpikeWindowWebcam spikeWindowWebcam = new SpikeWindowWebcam(frontWebcamConfiguration,
                Pair.create(RobotConstantsCenterStage.ProcessorIdentifier.SPIKE_WINDOW, spikeWindowProcessor));

        if (!spikeWindowWebcam.waitForWebcamStart(2000))
            throw new AutonomousRobotException(TAG, "Spike window webcam timed out on start");

        frontWebcamConfiguration.setVisionPortalWebcam(spikeWindowWebcam);
        RobotLog.ii(TAG, "SpikeWindowViewer successfully started on the front webcam");

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        try {
            SpikeWindowMappingXML spikeWindowMappingXML = new SpikeWindowMappingXML(xmlDirectory);
            collectedSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping();
        } catch (ParserConfigurationException | IOException | SAXException |
                 XPathExpressionException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }

        // Set up the DPAD buttons for starting position selection - clockwise
        // from the audience wall.
        startPositionA2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        startPositionA4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        startPositionF4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        startPositionF2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch play to *END* the OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayerOne();
        }

        telemetry.addLine("Ending the SpikeWindowViewer");
        telemetry.update();        
    }

    private void updateButtons() {
        startPositionF4Button.update();
        startPositionF2Button.update();
        startPositionA2Button.update();
        startPositionA4Button.update();
    }

    private void updatePlayerOne() {
        updateStartPositionF4Button();
        updateStartPositionF2Button();
        updateStartPositionA2Button();
        updateStartPositionA4Button();
    }

    private void updateStartPositionF4Button() {
        if (startPositionF4Button.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "DPAD button for F4 tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping f4SpikeWindows = collectedSpikeWindowMapping.get(RobotConstantsCenterStage.OpMode.RED_F4);
            if (f4SpikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping for F4 in the camera stream
            // on the Driver Station.
            spikeWindowProcessor.setSpikeWindowMapping(f4SpikeWindows);
            RobotLog.dd(TAG, "Set spike window mapping for F4");
            telemetry.addLine("Spike windows for RED_F4");
            telemetry.update();
        }
    }

    private void updateStartPositionF2Button() {
        if (startPositionF2Button.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "DPAD button for F2 tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping f2SpikeWindows = collectedSpikeWindowMapping.get(RobotConstantsCenterStage.OpMode.RED_F2);
            if (f2SpikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping for F2 in the camera stream
            // on the Driver Station.
            spikeWindowProcessor.setSpikeWindowMapping(f2SpikeWindows);
            RobotLog.dd(TAG, "Set spike window mapping for F2");
            telemetry.addLine("Spike windows for RED_F2");
            telemetry.update();
        }
    }

    private void updateStartPositionA2Button() {
        if (startPositionA2Button.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "DPAD button for A2 tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping a2SpikeWindows = collectedSpikeWindowMapping.get(RobotConstantsCenterStage.OpMode.BLUE_A2);
            if (a2SpikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping for A2 in the camera stream
            // on the Driver Station.
            spikeWindowProcessor.setSpikeWindowMapping(a2SpikeWindows);
            RobotLog.dd(TAG, "Set spike window mapping for A2");
            telemetry.addLine("Spike windows for BLUE_A2");
            telemetry.update();
        }
    }

    private void updateStartPositionA4Button() {
        if (startPositionA4Button.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "DPAD button for A4 tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping a4SpikeWindows = collectedSpikeWindowMapping.get(RobotConstantsCenterStage.OpMode.BLUE_A4);
            if (a4SpikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping for A4 in the camera stream
            // on the Driver Station.
            spikeWindowProcessor.setSpikeWindowMapping(a4SpikeWindows);
            RobotLog.dd(TAG, "Set spike window mapping for A4");
            telemetry.addLine("Spike windows for BLUE_A4");
            telemetry.update();
        }
    }

}