package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.StartParameters;
import org.firstinspires.ftc.teamcode.common.xml.StartParametersXML;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;

import javax.xml.parsers.ParserConfigurationException;

@TeleOp(name = "SetStartParameters", group = "Configure")
//@Disabled
public class SetStartParameters extends LinearOpMode {

    private static final String TAG = SetStartParameters.class.getSimpleName();

    private StartParametersXML startParametersXML;
    private StartParameters startParameters;

    private static final int MAX_START_DELAY = 10;
    private int startDelay;
    private int currentStartDelay;
    private FTCButton increaseDelay;
    private FTCButton decreaseDelay;

    EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;
    private boolean endPositionsChanged = false;
    private FTCButton endPositionLeft;
    private FTCButton endPositionRight;

    private FTCButton opModeRedF4;
    private FTCButton opModeRedF2;
    private FTCButton opModeBlueA2;
    private FTCButton opModeBlueA4;

    @Override
    public void runOpMode() {
        increaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        decreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        endPositionLeft = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_LEFT);
        endPositionRight = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_RIGHT);

        // Set up the DPAD buttons for starting position selection - clockwise
        // from the audience wall.
        opModeBlueA2 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        opModeBlueA4 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        opModeRedF4 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        opModeRedF2 = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        startParameters = startParametersXML.getStartParameters();
        currentStartDelay = startDelay = startParameters.autoStartDelay;

        telemetry.addLine("The current start delay is " + currentStartDelay);
        telemetry.addLine("DPAD_UP to increase delay; DPAD_DOWN to decrease");
        telemetry.addLine("Hold ABXY buttons to select OpMode:");
        telemetry.addLine("  A for BLUE_A2, X for BLUE_A4");
        telemetry.addLine("  Y for RED_F4, B for RED_F2");
        telemetry.addLine("Touch DPAD left or right for ending position");
        telemetry.addLine("Touch play to SAVE changes and END the OpMode");
        telemetry.update();

        autoEndingPositions = new EnumMap<>(startParameters.autoEndingPositions); // copy

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayer1();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } // while

        if (opModeIsActive()) {
            if (startDelay != currentStartDelay || endPositionsChanged) {
                if (startDelay != currentStartDelay) {
                    startParametersXML.setAutoStartDelay(currentStartDelay);
                    startParametersXML.writeStartParametersFile();
                    RobotLog.ii(TAG, "Changed start delay to " + currentStartDelay);
                    telemetry.addLine("Changed start delay to " + currentStartDelay);
                }

                if (endPositionsChanged) {
                    RobotLog.ii(TAG, "Changed one or more ending positions");
                    telemetry.addLine("Changed one or more ending positions");
                }

                startParametersXML.writeStartParametersFile();
                RobotLog.ii(TAG, "Writing StartParameters.xml");
                telemetry.addLine("Writing StartParameters.xml");
                telemetry.update();
                sleep(1500);
            }
        }
    }

    private void updateButtons() {
        increaseDelay.update();
        decreaseDelay.update();
        endPositionLeft.update();
        endPositionRight.update();
        opModeBlueA2.update();
        opModeBlueA4.update();
        opModeRedF4.update();
        opModeRedF2.update();
    }

    private void updatePlayer1() {
        updateIncreaseDelay();
        updateDecreaseDelay();
        updateOpModeBlueA2();
        updateOpModeBlueA4();
        updateOpModeRedF2();
        updateOpModeRedF4();
    }

    private void updateIncreaseDelay() {
        if (increaseDelay.is((FTCButton.State.TAP))) {
            currentStartDelay = currentStartDelay < MAX_START_DELAY ? ++currentStartDelay : MAX_START_DELAY;
            if (currentStartDelay == MAX_START_DELAY)
                telemetry.addLine("Start delay is at the maximum of " + MAX_START_DELAY);
            else
                telemetry.addLine("Start delay increased to " + currentStartDelay);
            telemetry.update();
        }
    }

    private void updateDecreaseDelay() {
        if (decreaseDelay.is((FTCButton.State.TAP))) {
            currentStartDelay = currentStartDelay >= 1 ? --currentStartDelay : 0; // 0 or positive
            if (currentStartDelay == 0)
                telemetry.addLine("Start delay is at the minimum of 0");
            else
                telemetry.addLine("Start delay decreased to " + currentStartDelay);
            telemetry.update();
        }
    }

    private void updateOpModeBlueA2() {
        setEndPosition(RobotConstantsCenterStage.OpMode.BLUE_A2, opModeBlueA2, endPositionLeft, endPositionRight);
    }

    private void updateOpModeBlueA4() {
        setEndPosition(RobotConstantsCenterStage.OpMode.BLUE_A4, opModeBlueA4, endPositionLeft, endPositionRight);
    }

    private void updateOpModeRedF2() {
        setEndPosition(RobotConstantsCenterStage.OpMode.RED_F2, opModeRedF2, endPositionLeft, endPositionRight);
    }

    private void updateOpModeRedF4() {
        setEndPosition(RobotConstantsCenterStage.OpMode.RED_F4, opModeRedF4, endPositionLeft, endPositionRight);
    }

    private void setEndPosition(RobotConstantsCenterStage.OpMode pOpMode, FTCButton pOpModeButton,
                                FTCButton pEndPositionLeftButton, FTCButton pEndPositionRightButton) {
        if (pOpModeButton.is(FTCButton.State.TAP) || pOpModeButton.is(FTCButton.State.HELD)) {
            if (pOpModeButton.is(FTCButton.State.TAP)) {
                // Display the current end position.
                RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(pOpMode);
                telemetry.addLine(pOpMode + " end position: " + endPosition);
                telemetry.update();
                return;
            }

            // Button is held - check for a LEFT or RIGHT selection from the DPAD.
            RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(pOpMode);
            if (pEndPositionLeftButton.is(FTCButton.State.TAP)) {
                if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.LEFT) {
                    telemetry.addLine("Ending position is already set to LEFT for " + pOpMode);
                    telemetry.update();
                    return;
                }

                endPositionsChanged = true;
                startParametersXML.setAutoEndingPosition(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT.toString());
                autoEndingPositions.put(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT);

                telemetry.addLine("Ending position set to LEFT for " + pOpMode);
                telemetry.update();
            } else if (pEndPositionRightButton.is(FTCButton.State.TAP)) {
                if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.RIGHT) {

                    telemetry.addLine("Ending position is already set to RIGHT for " + pOpMode);
                    telemetry.update();
                    return;
                }

                endPositionsChanged = true;
                startParametersXML.setAutoEndingPosition(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT.toString());
                autoEndingPositions.put(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT);
                telemetry.addLine("Ending position set to RIGHT for " + pOpMode);
                telemetry.update();
            }
        }
    }
}