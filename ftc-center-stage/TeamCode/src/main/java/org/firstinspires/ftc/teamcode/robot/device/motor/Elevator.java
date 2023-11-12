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

    //**TODO Remove CLEAR from all locations, including RobotConfig.xml - not needed without the boom
    public enum ElevatorLevel {
        GROUND, SAFE, CLEAR, AUTONOMOUS, LEVEL_1, LEVEL_2, LEVEL_3
    }

    public static final int ELEVATOR_MIN_POSITION = 0;
    public static final int ELEVATOR_MAX_POSITION = 8700;

    public final int ground;
    public final int safe;
    public final int clear;
    public final int autonomous;
    public final int level_1;
    public final int level_2;
    public final int level_3;

    // There are two elevator motors that operate in tandem.
    public Elevator(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.ELEVATOR_LEFT, FTCRobot.MotorId.ELEVATOR_RIGHT);

         /*
           <positions>
             <ground>0</ground>
             <safe>0</safe>
             <clear>0</clear>
             <autonomous>0</autonomous>
             <level_1>0</level_1>
             <level_2>0</Level_2>
             <level_3>0</level_3>
           </positions>
         */

        ground = pConfigXPath.getRequiredInt("positions/ground");
        if (ground < ELEVATOR_MIN_POSITION || ground > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator ground position is out of range");

        safe = pConfigXPath.getRequiredInt("positions/safe");
        if (safe < ELEVATOR_MIN_POSITION || safe > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator safe position is out of range");

        clear = pConfigXPath.getRequiredInt("positions/clear");
        if (clear < ELEVATOR_MIN_POSITION || clear > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator clear position is out of range");

        autonomous = pConfigXPath.getRequiredInt("positions/autonomous");
        if (autonomous < ELEVATOR_MIN_POSITION || autonomous > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator autonomous position is out of range");

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