package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.StartParameters;
import org.firstinspires.ftc.teamcode.common.StartParametersXML;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

//**TODO Extend to include Auto ending positions. Use both gamepads.

@TeleOp(name = "SetStartParameters", group = "Configure")
//@Disabled
public class SetStartParameters extends LinearOpMode {

    private static final String TAG = SetStartParameters.class.getSimpleName();

    private FTCButton increaseDelay;
    private FTCButton decreaseDelay;
    private int startDelay;
    private int currentStartDelay;

    private FTCButton endPositionLeft;
    private FTCButton endPositionRight;

    private FTCButton startPositionF4Button;
    private FTCButton startPositionF2Button;
    private FTCButton startPositionA2Button;
    private FTCButton startPositionA4Button;

    @Override
    public void runOpMode() {
        increaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        decreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        endPositionLeft = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_LEFT);
        endPositionRight = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_RIGHT);

        // Set up the DPAD buttons for starting position selection - clockwise
        // from the audience wall.
        startPositionA2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        startPositionA4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        startPositionF4Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        startPositionF2Button = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        StartParametersXML startParametersXML;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        StartParameters startParameters = startParametersXML.getStartParameters();
        currentStartDelay = startDelay = startParameters.autoStartDelay;

        telemetry.addLine("The current start delay is " + currentStartDelay);
        telemetry.addLine("DPAD_UP to increase delay; DPAD_DOWN to decrease");
        telemetry.addLine("Touch play to SAVE the delay and END the OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayer1();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } // while


        //**TODO if isActive()
        //**TODO if change in endPosition
        if (startDelay != currentStartDelay) {
            startParametersXML.setAutoStartDelay(startDelay);
            startParametersXML.writeStartParametersFile();
            RobotLog.ii(TAG, "Writing StartParameters.xml with an Autonomous start delay of " + startDelay);
            telemetry.addLine("Writing StartParameters.xml");
            telemetry.update();
            sleep(1500);
        }
    }

    private void updateButtons() {
        increaseDelay.update();
        decreaseDelay.update();
        endPositionLeft.update();
        endPositionRight.update();
        startPositionA2Button.update();
        startPositionA4Button.update();
        startPositionF4Button.update();
        startPositionF2Button.update();
    }

    private void updatePlayer1() {
        updateIncreaseDelay();
        updateDecreaseDelay();
    }

    private void updateIncreaseDelay() {
        if (increaseDelay.is((FTCButton.State.TAP))) {
            startDelay = startDelay < 5 ? ++startDelay : 5; // 5 sec max
        }
    }

    private void updateDecreaseDelay() {
        if (decreaseDelay.is((FTCButton.State.TAP))) {
            startDelay = startDelay >= 1 ? --startDelay : 0; // 0 or positive
        }
    }

    //**TODO two simultaneous button pushes (hold + tap) to set end position

}