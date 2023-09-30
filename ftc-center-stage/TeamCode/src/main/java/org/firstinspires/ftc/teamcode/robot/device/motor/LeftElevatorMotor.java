package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Easy way to test a single motor when the RobotController
// is configured for two elevator motors. Rename this class
// and add XML parameters to use it for a different single
// motor.
public class LeftElevatorMotor extends SingleMotor {

    public static final String TAG = LeftElevatorMotor.class.getSimpleName();

    public final double velocity;

    // There are two elevator motors that operate in tandem.
    public LeftElevatorMotor(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.ELEVATOR_LEFT);

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);
    }

}