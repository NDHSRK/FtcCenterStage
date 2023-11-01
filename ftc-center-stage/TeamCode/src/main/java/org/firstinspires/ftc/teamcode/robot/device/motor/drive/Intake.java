package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualSPARKMiniMotorControllers;

import javax.xml.xpath.XPathExpressionException;

public class Intake extends DualSPARKMiniMotorControllers {

    // The intake/outtake for picking up pixels and/or delivering them to the
    // spike or the backdrop.
    public Intake(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath);
    }

    //  controller1 = hardwareMap.get(DcMotorSimple.class, "front_intake");
    //  controller2 = hardwareMap.get(DcMotorSimple.class, "main_intake");
}