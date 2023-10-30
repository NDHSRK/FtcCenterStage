package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

public class Boom extends SingleMotor {

    public static final String TAG = Boom.class.getSimpleName();
 
    public enum BoomLevel {
        REST, LEVEL_1, LEVEL_2, LEVEL_3
    }

    public static final int BOOM_MIN_POSITION = 0;
    public static final int BOOM_MAX_POSITION = 2690;
 
    public final double velocity;

    public final int rest;
    public final int level_1;
    public final int level_2;
    public final int level_3;

    // The boom for delivering pixels to the backdrop.
    public Boom(HardwareMap pHardwareMap, XPathAccess pConfigXPath) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, FTCRobot.MotorId.BOOM);

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);
        
         /*
           <positions>
             <rest>0</rest>
             <level_1>0</level_1>
             <level_2>0</Level_2>
             <level_3>0</level_3>
           </positions>
         */

        rest = pConfigXPath.getRequiredInt("positions/rest");
        if (rest < BOOM_MIN_POSITION || rest > BOOM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator rest position is out of range");

        level_1 = pConfigXPath.getRequiredInt("positions/level_1");
        if (level_1 < BOOM_MIN_POSITION || level_1 > BOOM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_1 position is out of range");

        level_2 = pConfigXPath.getRequiredInt("positions/level_2");
        if (level_2 < BOOM_MIN_POSITION || level_2 > BOOM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_2 position is out of range");

        level_3 = pConfigXPath.getRequiredInt("positions/level_3");
        if (level_3 < BOOM_MIN_POSITION || level_3 > BOOM_MAX_POSITION)
            throw new AutonomousRobotException(TAG, "Elevator level_3 position is out of range");
    }

}