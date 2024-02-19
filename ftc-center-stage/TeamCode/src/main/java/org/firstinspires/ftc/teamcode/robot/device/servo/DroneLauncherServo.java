package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class DroneLauncherServo {

    public enum DroneLauncherState {HOLD, LAUNCH};
    public final Servo servo;
    private final double hold;
    private final double launch;

    public DroneLauncherServo(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        String TAG = DroneLauncherServo.class.getSimpleName();

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Drone Launcher Servo configuration from RobotConfig.xml");
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("servo/@model"));

        servo = pHardwareMap.get(Servo.class, pConfigXPath.getRequiredText("servo/id"));

        // Get the servo positions
        /*
        <positions>
            <hold>
            <launch>
        </positions>
         */

        hold = pConfigXPath.getRequiredDouble("positions/hold");
        launch = pConfigXPath.getRequiredDouble("positions/launch");
    }

    public void hold(){
        servo.setPosition(hold);
    }

    public void launch(){
        servo.setPosition(launch);
    }

}
