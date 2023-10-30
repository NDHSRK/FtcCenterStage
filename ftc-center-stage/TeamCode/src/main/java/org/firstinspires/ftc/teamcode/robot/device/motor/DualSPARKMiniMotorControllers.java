package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

// Support two Rev SPARKMini motor controllers that can be used in tandem in both
// Autonomous and TeleOp.
public abstract class DualSPARKMiniMotorControllers {

    private final String TAG = DualSPARKMiniMotorControllers.class.getSimpleName();

    protected final DcMotorSimple leftController;
    protected final DcMotorSimple rightController;

    public DualSPARKMiniMotorControllers(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining dual SPARKMini motor controllers");
        RobotLogCommon.c(TAG, "Controllers " + pConfigXPath.getRequiredText("servos/@model"));

        leftController = pHardwareMap.get(DcMotorSimple.class, pConfigXPath.getRequiredText("servos/left_controller"));
        leftController.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("servos/left_controller/@direction")));

        rightController = pHardwareMap.get(DcMotorSimple.class, pConfigXPath.getRequiredText("servos/right_controller"));
        rightController.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("servos/right_controller/@direction")));
    }

    public void setPower(double pLeftPower, double pRightPower) {
        leftController.setPower(pLeftPower);
        rightController.setPower(pRightPower);
    }

    public void stop() {
        leftController.setPower(0);
        rightController.setPower(0);
    }

}