package org.firstinspires.ftc.teamcode.robot.device.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;

import javax.xml.xpath.XPathExpressionException;

public class IntakeArmServo {
    public enum IntakeArmState {UP, DOWN, STACK}

    // Allow public access for testing.
    public final Servo leftServo;
    public final Servo rightServo;

    private final double set_up;
    private final double set_down;
    private final double set_stack;

    private IntakeArmState intakeArmState;

    public IntakeArmServo(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        String TAG = IntakeArmServo.class.getSimpleName();

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Intake/Outtake Servo configuration from RobotConfig.xml");
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("servos/@model"));

        leftServo = pHardwareMap.get(Servo.class, pConfigXPath.getRequiredText("servos/left_arm"));
        rightServo = pHardwareMap.get(Servo.class, pConfigXPath.getRequiredText("servos/right_arm"));

        // Get the servo positions
        /*
        <positions>
            <up></up>
            <down></down>
            <stack></stack>
        </positions>
         */

        set_up = pConfigXPath.getRequiredDouble("positions/up");
        set_down = pConfigXPath.getRequiredDouble("positions/down");
        set_stack = pConfigXPath.getRequiredDouble("positions/stack");

        // Always start with the intake arm in the up position.
        up();
    }

    public void up() {
        leftServo.setPosition(0.5 + set_up);
        rightServo.setPosition(0.5 - set_up);
        intakeArmState = IntakeArmState.UP;
    }

    public void down() {
        leftServo.setPosition(0.5 - set_down);
        rightServo.setPosition(0.5 + set_down);
        intakeArmState = IntakeArmState.DOWN;
    }

    public void stack() {
        leftServo.setPosition(0.5 - set_stack);
        rightServo.setPosition(0.5 + set_stack);
        intakeArmState = IntakeArmState.STACK;
    }

    public IntakeArmState getIntakeArmState() {
        return intakeArmState;
    }

}
