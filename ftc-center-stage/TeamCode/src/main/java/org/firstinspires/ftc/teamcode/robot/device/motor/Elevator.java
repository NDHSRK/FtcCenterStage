package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Center Stage Elevator Motors. These motors are used both in Autonomous
// and TeleOp. Users of this class must ensure that the motors are in the
// correct DcMotor.RunMode. See comments in MotorCore.
public class Elevator extends DualMotors {

    public static final String TAG = Elevator.class.getSimpleName();

    public enum ElevatorLevel {
        GROUND, SAFE, PIXEL_CLEARANCE, DRONE, AUTONOMOUS, LEVEL_1, LEVEL_2, LEVEL_3,
        ON_TRUSS, ABOVE_TRUSS
    }

    public static final int ELEVATOR_MIN_POSITION = 0;
    public static final int ELEVATOR_MAX_POSITION = 6200;

    public final double velocity_down;
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
    public Elevator(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.ELEVATOR_LEFT, FTCRobot.MotorId.ELEVATOR_RIGHT);

        // The base class parses the >velocity> element but for coordination
        // with the winch we need a separate velocity setting for downward
        // movement.
        velocity_down = pConfigXPath.getRequiredDouble("velocity_down");
        if (velocity_down <= 0.0 || velocity_down > 1.0)
            throw new AutonomousRobotException(TAG, "velocity_down out of range " + velocity_down);

        ground = pConfigXPath.getRequiredInt("positions/ground");
        if (ground < ELEVATOR_MIN_POSITION || ground > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator ground position is out of range");

        safe = pConfigXPath.getRequiredInt("positions/safe");
        if (safe < ELEVATOR_MIN_POSITION || safe > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator safe position is out of range");

        pixel_clearance = pConfigXPath.getRequiredInt("positions/pixel_clearance");
        if (safe < ELEVATOR_MIN_POSITION || safe > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator pixel_clearance position is out of range");

        drone = pConfigXPath.getRequiredInt("positions/drone");
        if (drone < ELEVATOR_MIN_POSITION || drone > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator drone position is out of range");

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

        on_truss = pConfigXPath.getRequiredInt("positions/on_truss");
        if (on_truss < ELEVATOR_MIN_POSITION || on_truss > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator on_truss position is out of range");

        above_truss = pConfigXPath.getRequiredInt("positions/above_truss");
        if (above_truss < ELEVATOR_MIN_POSITION || above_truss > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator above_truss position is out of range");
    }

}