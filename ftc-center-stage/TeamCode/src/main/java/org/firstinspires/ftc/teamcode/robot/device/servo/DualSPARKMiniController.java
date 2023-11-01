package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

// Support two Rev SPARKMini motor controllers that can be used in tandem in both
// Autonomous and TeleOp.
public abstract class DualSPARKMiniController {

    private final String TAG = DualSPARKMiniController.class.getSimpleName();

    protected final DcMotorSimple controller1;
    protected final DcMotorSimple controller2;
    private double power;

    public DualSPARKMiniController(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining dual SPARKMini motor controllers");
        RobotLogCommon.c(TAG, "Controllers " + pConfigXPath.getRequiredText("servos/@model"));

        controller1 = pHardwareMap.get(DcMotorSimple.class, pConfigXPath.getRequiredText("servos/controller_1"));
        controller1.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("servos/controller_1/@direction")));

        controller2 = pHardwareMap.get(DcMotorSimple.class, pConfigXPath.getRequiredText("servos/controller_2"));
        controller2.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("servos/controller_2/@direction")));

        power = pConfigXPath.getRequiredDouble("power");
        if (power <= 0.0 || power > 1.0)
            throw new AutonomousRobotException(TAG, "power out of range " + power);
    }

    public void runWithCurrentPower() {
        controller1.setPower(power);
        controller2.setPower(power);
    }

    public void setPower(double pPower) {
        power = Math.abs(pPower);
    }

    public double getPower() {
        return power;
    }

    public void stop() {
        controller1.setPower(0);
        controller2.setPower(0);
    }

}