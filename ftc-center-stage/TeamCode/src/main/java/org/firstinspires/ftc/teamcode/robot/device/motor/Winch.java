package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Center Stage Winch Motor. This motor is used in the TeleOp end game.
// Users of this class must ensure that the motor is in the correct
// DcMotor.RunMode. See comments in MotorCore.
public class Winch extends SingleMotor {

    public static final String TAG = Winch.class.getSimpleName();

    public enum WinchLevel {
        GROUND, SAFE, PIXEL_CLEARANCE, DRONE, AUTONOMOUS, LEVEL_1, LEVEL_2, LEVEL_3,
        ON_TRUSS, ABOVE_TRUSS
    }

    public static final int WINCH_MIN_POSITION = -10000; //**TODO TBD
    public static final int WINCH_MAX_POSITION = 10000; //**TODO TBD

    public final int ground;
    public final int safe;
    public final int pixel_clearance;
    public final int drone;
    public final int autonomous;
    public final int level_1;
    public final int level_2;
    public final int level_3;
    public final int on_truss;
    public final int above_truss;

    // There are two elevator motors that operate in tandem.
    public Winch(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.WINCH);

        ground = pConfigXPath.getRequiredInt("positions/ground");
        if (ground < WINCH_MIN_POSITION || ground > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator ground position is out of range");

        safe = pConfigXPath.getRequiredInt("positions/safe");
        if (safe < WINCH_MIN_POSITION || safe > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator safe position is out of range");

        pixel_clearance = pConfigXPath.getRequiredInt("positions/pixel_clearance");
        if (safe < WINCH_MIN_POSITION || safe > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator pixel_clearance position is out of range");

        drone = pConfigXPath.getRequiredInt("positions/drone");
        if (drone < WINCH_MIN_POSITION || drone > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator drone position is out of range");

        autonomous = pConfigXPath.getRequiredInt("positions/autonomous");
        if (autonomous < WINCH_MIN_POSITION || autonomous > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator autonomous position is out of range");

        level_1 = pConfigXPath.getRequiredInt("positions/level_1");
        if (level_1 < WINCH_MIN_POSITION || level_1 > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_1 position is out of range");

        level_2 = pConfigXPath.getRequiredInt("positions/level_2");
        if (level_2 < WINCH_MIN_POSITION || level_2 > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_2 position is out of range");

        level_3 = pConfigXPath.getRequiredInt("positions/level_3");
        if (level_3 < WINCH_MIN_POSITION || level_3 > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_3 position is out of range");

        on_truss = pConfigXPath.getRequiredInt("positions/on_truss");
        if (on_truss < WINCH_MIN_POSITION || on_truss > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator on_truss position is out of range");

        above_truss = pConfigXPath.getRequiredInt("positions/above_truss");
        if (above_truss < WINCH_MIN_POSITION || above_truss > WINCH_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator above_truss position is out of range");
    }

}