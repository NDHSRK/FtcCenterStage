package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.StartParametersXML;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

@TeleOp(name = "SetAutoStartDelay", group = "Configure")
//@Disabled
public class SetAutoStartDelay extends LinearOpMode {

    private static final String TAG = SetAutoStartDelay.class.getSimpleName();

    @Override
    public void runOpMode() {
        FTCButton increaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        FTCButton decreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        StartParametersXML startParametersXML;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        int startDelay;
        int currentStartDelay = startDelay = startParametersXML.getAutoStartDelay();
        telemetry.addLine("The current start delay is " + currentStartDelay);
        telemetry.addLine("DPAD_UP to increase delay; DPAD_DOWN to decrease");
        telemetry.addLine("Touch play to *END* the OpMode");
        telemetry.update();

        boolean changeInDelay = false;
        while (!isStarted() && !isStopRequested()) {
            increaseDelay.update();
            decreaseDelay.update();
            if (increaseDelay.is((FTCButton.State.TAP))) {
                startDelay = startDelay < 5 ? ++startDelay : 5; // 5 sec max
                changeInDelay = true;
            } else if (decreaseDelay.is((FTCButton.State.TAP))) {
                startDelay = startDelay >= 1 ? --startDelay : 0; // 0 or positive
                changeInDelay = true;
            }

            if (changeInDelay) {
                changeInDelay = false;
                telemetry.addLine("Start delay changed to " + startDelay + " sec");
                telemetry.update();
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } // while


        if (startDelay != currentStartDelay) {
            startParametersXML.setAutoStartDelay(startDelay);
            RobotLog.ii(TAG, "Writing StartParameters.xml with an Autonomous start delay of " + startDelay);
            telemetry.addLine("Writing StartParameters.xml");
            telemetry.update();
            sleep(1500);
        }
    }

}