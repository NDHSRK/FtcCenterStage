package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class PixelIO extends DualSPARKMiniController {

    // The intake/outtake for picking up pixels from the front of the robot
    // delivering them back out the front to the spike or out the back to
    // the backdrop.
    public PixelIO(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath);
    }

}