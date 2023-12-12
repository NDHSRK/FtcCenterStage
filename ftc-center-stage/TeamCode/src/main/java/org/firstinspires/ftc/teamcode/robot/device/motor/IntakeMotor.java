package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;

import javax.xml.xpath.XPathExpressionException;

// Motor for the intake/outtake motor.
// Users of this class must ensure that the motor is in the correct
// DcMotor.RunMode. See comments in MotorCore.
public class IntakeMotor extends SingleMotor {

    public static final String TAG = IntakeMotor.class.getSimpleName();

    public static final double MINIMUM_ARM_MOTOR_VELOCITY = .2;
    public static final int ARM_MIN_POSITION = 0;
    public static final int ARM_MAX_POSITION = 2000;

    public enum IntakeArmPosition {REST, INTAKE, DELIVER_FRONT, DELIVER_BACK}

    public final double velocity;
    public final int rest;
    public final int intake;
    public final int deliver_front;
    public final int deliver_back;

    public IntakeMotor(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.INTAKE);

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);

        // Get the motor positions
        /*
        <positions>
            <rest>0</rest>
            <intake>0</intake>
            <deliver_front>0</deliver_front>
            <deliver_back>0</deliver_back>
        </positions>
         */
        rest = pConfigXPath.getRequiredInt("positions/rest");
        if (rest < ARM_MIN_POSITION || rest > ARM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Arm rest position is out of range");

        intake = pConfigXPath.getRequiredInt("positions/intake");
        if (intake < ARM_MIN_POSITION || intake > ARM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Intake position is out of range");

        deliver_front = pConfigXPath.getRequiredInt("positions/deliver_front");
        if (deliver_front < ARM_MIN_POSITION || deliver_front > ARM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "deliver_front position is out of range");

        deliver_back = pConfigXPath.getRequiredInt("positions/deliver_back");
        if (deliver_back < ARM_MIN_POSITION || deliver_back > ARM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "deliver_back position is out of range");
    }

}