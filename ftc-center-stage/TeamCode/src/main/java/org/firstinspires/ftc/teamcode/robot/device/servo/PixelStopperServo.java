package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class PixelStopperServo {

    public enum PixelServoState {HOLD, RELEASE};
    public final Servo servo;
    private final double hold;
    private final double release;

    public PixelStopperServo(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        String TAG = PixelStopperServo.class.getSimpleName();

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Pixel Stopper Servo configuration from RobotConfig.xml");
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("servo/@model"));

        servo = pHardwareMap.get(Servo.class, pConfigXPath.getRequiredText("servo/id"));

        // Get the servo positions
        /*
        <positions>
            <hold>
            <release>
        </positions>
         */

        hold = pConfigXPath.getRequiredDouble("positions/hold");
        release = pConfigXPath.getRequiredDouble("positions/release");
    }

    public void hold(){
        servo.setPosition(hold);
    }

    public void release(){
        servo.setPosition(release);
    }

}
