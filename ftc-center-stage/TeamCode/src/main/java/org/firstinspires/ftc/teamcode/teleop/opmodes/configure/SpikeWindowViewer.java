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
        setSpikeWindowMapping(RobotConstantsCenterStage.OpMode.BLUE_A2, opModeBlueA2);
    }

    private void updateOpModeBlueA4() {
        setSpikeWindowMapping(RobotConstantsCenterStage.OpMode.BLUE_A4, opModeBlueA4);
    }

    private void updateOpModeRedF4() {
        setSpikeWindowMapping(RobotConstantsCenterStage.OpMode.RED_F4, opModeRedF4);
    }

    private void updateOpModeRedF2() {
        setSpikeWindowMapping(RobotConstantsCenterStage.OpMode.RED_F2, opModeRedF2);
    }

    private void setSpikeWindowMapping(RobotConstantsCenterStage.OpMode pOpMode, FTCButton pOpModeButton) {
        if (pOpModeButton.is(FTCButton.State.TAP)) {
            RobotLog.dd(TAG, "DPAD button for " + pOpMode + " tapped");

            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping spikeWindows = collectedSpikeWindowMapping.get(pOpMode);
            if (spikeWindows == null)
                return; // ignore the button click

            // Show the spike window mapping for A4 in the camera stream
            // on the Driver Station.
            spikeWindowProcessor.setSpikeWindowMapping(spikeWindows);
            RobotLog.dd(TAG, "Set spike window mapping for " + pOpMode);
            telemetry.addLine("Spike windows for " + pOpMode);
            telemetry.update();
        }
    }

}