package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class IntakeArmHolderServo {

    public final Servo servo;
    private final double hold;
    private final double release;

    public IntakeArmHolderServo(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        String TAG = IntakeArmHolderServo.class.getSimpleName();

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Intake/Outtake Servo configuration from RobotConfig.xml");
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
