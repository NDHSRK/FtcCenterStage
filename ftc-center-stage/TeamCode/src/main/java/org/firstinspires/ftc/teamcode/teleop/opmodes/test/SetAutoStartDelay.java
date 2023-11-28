package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;

import java.util.SortedSet;

@TeleOp(name = "SetAutoStartDelay", group = "Test")
//@Disabled
public class SetAutoStartDelay extends LinearOpMode {

    private static final String TAG = SetAutoStartDelay.class.getSimpleName();
    private int startDelay = 0;
    FTCButton increaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);
    FTCButton decreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_2_DPAD_DOWN);

    @Override
    public void runOpMode() {

        telemetry.addLine("DPAD_UP to increase delay; DPAD_DOWN to decrease");
        telemetry.addLine("Touch play to *END* the OpMode");
        telemetry.update();

        boolean changeInDelay = false;
        while (!isStarted() && !isStopRequested()) {
            increaseDelay.update();
            decreaseDelay.update();
            if (increaseDelay.is((FTCButton.State.TAP))) {
                startDelay = startDelay < 5000 ? startDelay + 1000 : 5000; // ms; 5 sec max
                changeInDelay = true;
            } else if (decreaseDelay.is((FTCButton.State.TAP))) {
                startDelay = startDelay >= 1000 ? startDelay - 1000 : 0; // ms; 0 or positive
                changeInDelay = true;
            }

            if (changeInDelay) {
                changeInDelay = false;
                telemetry.addLine("Start delay changed to" + startDelay / 1000);
                telemetry.update();
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } // while

        if (startDelay != 0) {
            //**TODO write out the StartDelay.xml file with the new value.
            telemetry.addLine("Writing StartAutoDelay.xml");
            telemetry.update();
            sleep(1500);
        }
    }

}