package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Freight Frenzy Elevator Motors. These motors are used both in Autonomous
// and TeleOp. Users of this class must ensure that the motors are in the
// correct DcMotor.RunMode. See comments in MotorCore.
public class Elevator extends DualMotors {

    public static final String TAG = Elevator.class.getSimpleName();

    public enum ElevatorLevel {
        REST, SAFE, LEVEL_1, LEVEL_2, LEVEL_3
    }

    public static final int ELEVATOR_MIN_POSITION = 0;
    public static final int ELEVATOR_MAX_POSITION = 3500; //**TODO

    public final double velocity;

    public final int rest;
    public final int safe;
    public final int level_1;
    public final int level_2;
    public final int level_3;

    // There are two elevator motors that operate in tandem.
    public Elevator(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.ELEVATOR_LEFT, FTCRobot.MotorId.ELEVATOR_RIGHT);

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);

         /*
           <positions>
             <rest>0</rest>
             <safe>0</safe>
             <level_1>0</level_1>
             <level_2>0</Level_2>
             <level_3>0</level_3>
           </positions>
         */

        rest = pConfigXPath.getRequiredInt("positions/rest");
        if (rest < ELEVATOR_MIN_POSITION || rest > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator rest position is out of range");

        safe = pConfigXPath.getRequiredInt("positions/safe");
        if (safe < ELEVATOR_MIN_POSITION || safe > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator safe position is out of range");

        level_1 = pConfigXPath.getRequiredInt("positions/level_1");
        if (level_1 < ELEVATOR_MIN_POSITION || level_1 > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_1 position is out of range");

        level_2 = pConfigXPath.getRequiredInt("positions/level_2");
        if (level_2 < ELEVATOR_MIN_POSITION || level_2 > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_2 position is out of range");

        level_3 = pConfigXPath.getRequiredInt("positions/level_3");
        if (level_3 < ELEVATOR_MIN_POSITION || level_3 > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_3 position is out of range");
    }

}