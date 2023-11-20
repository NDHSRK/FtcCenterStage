package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.auto.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

// This OpMode gives the drive team a way to check the
// alignment of the front camera to make sure that the
// Team Prop is in view.
@TeleOp(name = "SpikeWindowViewer", group = "Test")
@Disabled
public class SpikeWindowViewer extends TeleOpBase {
    private static String TAG = SpikeWindowViewer.class.getSimpleName();

    private EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowMapping> collectedSpikeWindowMapping;
    private FTCButton startPositionF4Button;
    private FTCButton startPositionF2Button;
    private FTCButton startPositionA2Button;
    private FTCButton startPositionA4Button;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    // In this OpMode all of the action takes place during init().
    @Override
    public void initialize() {

        String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        try {
            SpikeWindowMappingXML spikeWindowMappingXML = new SpikeWindowMappingXML(xmlDirectory);
            collectedSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping();
        } catch (ParserConfigurationException | IOException | SAXException | XPathExpressionException ex) {
            throw new AutonomousRobotException(TAG, ex.getMessage());
        }

        // Set up the DPAD buttons for starting position selection - clockwise
        // from the wall with the two backdrops.
        startPositionF4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_LEFT);
        startPositionF2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        startPositionA2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_RIGHT);
        startPositionA4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch play to *END* the OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayerOne();
        }
    }

    @Override
    public void run() {
        if (opModeIsActive()) {
            telemetry.addLine("Ending the OpMode");
            telemetry.update();
        }
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
            // Make sure that the Autonomous OpMode for the selected
            // starting position has actually been defined in RobotAction.xml.
            SpikeWindowMapping f4SpikeWindows = collectedSpikeWindowMapping.get(RobotConstantsCenterStage.OpMode.RED_F4);
            if (f4SpikeWindows == null)
                return; // ignore the button click

            //**TODO set the spike window mapping in the SpikeWindowProcessot ...
        }
    }

    private void updateStartPositionF2Button() {
        if (startPositionF2Button.is(FTCButton.State.TAP)) {
        }
    }

    private void updateStartPositionA2Button() {
        if (startPositionA2Button.is(FTCButton.State.TAP)) {
        }
    }

    private void updateStartPositionA4Button() {
        if (startPositionA4Button.is(FTCButton.State.TAP)) {
        }
    }

}