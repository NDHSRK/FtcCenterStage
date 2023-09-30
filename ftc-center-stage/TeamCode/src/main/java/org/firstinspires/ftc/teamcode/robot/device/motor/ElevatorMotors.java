package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Freight Frenzy Elevator Motors. These motors are used both in Autonomous
// and TeleOp. Users of this class must ensure that the motors are in the
// correct DcMotor.RunMode. See comments in MotorCore.
public class ElevatorMotors extends DualMotors {

    public static final String TAG = ElevatorMotors.class.getSimpleName();

    public enum ElevatorLevel {
        DOWN, SAFE, LOW_JUNCTION, MEDIUM_JUNCTION, HIGH_JUNCTION,
        CONE_ONE, CONE_TWO, CONE_THREE, CONE_FOUR, CONE_FIVE,
        STACK_CLEARANCE, TEST
    }

    public static final int ELEVATOR_MIN_POSITION = 0;
    public static final int ELEVATOR_MAX_POSITION = 3500;
    public static final int ELEVATOR_MIN_RELATIVE_DESCENT = 0;
    public static final int ELEVATOR_MAX_RELATIVE_DESCENT = 600;

    public final double velocity;
    public final int down;
    public final int safe;
    public final int low_junction;
    public final int medium_junction;
    public final int high_junction;
    public final int cone_one;
    public final int cone_two;
    public final int cone_three;
    public final int cone_four;
    public final int cone_five;
    public final int stack_clearance;
    public final int relative_descent;

    // There are two elevator motors that operate in tandem.
    public ElevatorMotors(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.ELEVATOR_LEFT, FTCRobot.MotorId.ELEVATOR_RIGHT);

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);

         /*
            <positions>
                <down>
                <safe>
                <low_junction>
                <medium_junction>
                <high_junction>
                <cone_one>
                <cone_two>
                <cone_three>
                <cone_four>
                <cone_five>
                <relative_descent>
            </positions>
         */
        down = pConfigXPath.getRequiredInt("positions/down");
        if (down < ELEVATOR_MIN_POSITION || down > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator down position is out of range");

        safe = pConfigXPath.getRequiredInt("positions/safe");
        if (safe < ELEVATOR_MIN_POSITION || safe > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator safe position is out of range");

        low_junction = pConfigXPath.getRequiredInt("positions/low_junction");
        if (low_junction < ELEVATOR_MIN_POSITION || low_junction > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator low_junction position is out of range");

        medium_junction = pConfigXPath.getRequiredInt("positions/medium_junction");
        if (medium_junction < ELEVATOR_MIN_POSITION || medium_junction > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator medium_junction position is out of range");

        high_junction = pConfigXPath.getRequiredInt("positions/high_junction");
        if (high_junction < ELEVATOR_MIN_POSITION || high_junction > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator high_junction position is out of range");

        cone_one = pConfigXPath.getRequiredInt("positions/cone_one");
        if (cone_one < ELEVATOR_MIN_POSITION || cone_one > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator cone_one position is out of range");

        cone_two = pConfigXPath.getRequiredInt("positions/cone_two");
        if (cone_two < ELEVATOR_MIN_POSITION || cone_two > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator cone_two position is out of range");

        cone_three = pConfigXPath.getRequiredInt("positions/cone_three");
        if (cone_three < ELEVATOR_MIN_POSITION || cone_three > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator cone_three position is out of range");

        cone_four = pConfigXPath.getRequiredInt("positions/cone_four");
        if (cone_four < ELEVATOR_MIN_POSITION || cone_four > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator cone_four position is out of range");

        cone_five = pConfigXPath.getRequiredInt("positions/cone_five");
        if (cone_five < ELEVATOR_MIN_POSITION || cone_five > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator cone_five position is out of range");

        stack_clearance = pConfigXPath.getRequiredInt("positions/stack_clearance");
        if (stack_clearance < ELEVATOR_MIN_POSITION || stack_clearance > ELEVATOR_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator stack_clearance position is out of range");

        relative_descent = pConfigXPath.getRequiredInt("positions/relative_descent");
        if (relative_descent < ELEVATOR_MIN_RELATIVE_DESCENT || relative_descent > ELEVATOR_MAX_RELATIVE_DESCENT)
            throw new AutonomousRobotException(TAG, "Elevator relative_descent position is out of range");
    }

}