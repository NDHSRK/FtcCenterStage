package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;

import javax.xml.xpath.XPathExpressionException;

// Support two motors that can be used in tandem in both Autonomous and TeleOp.
// Users of this class must ensure that the motor is in the correct DcMotor.RunMode.
// See comments in MotorCore.
public abstract class DualMotors extends MotorCore {

    private final String TAG = DualMotors.class.getSimpleName();

    protected final FTCRobot.MotorId leftMotorId;
    protected final FTCRobot.MotorId rightMotorId;
    private double velocity; // This is the configured velocity from RobotConfig.xml

    public DualMotors(HardwareMap pHardwareMap, XPathAccess pConfigXPath, FTCRobot.MotorId pLeftMotorId, FTCRobot.MotorId pRightMotorId) throws XPathExpressionException {
        super(pConfigXPath, "dual_motors");
        leftMotorId = pLeftMotorId;
        rightMotorId = pRightMotorId;

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining dual motors " + pLeftMotorId + " and " + pRightMotorId);
        RobotLogCommon.c(TAG, "Motors " + pConfigXPath.getRequiredText("dual_motors/@model"));

        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        DcMotorEx left_motor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("dual_motors/left_motor"));
       
        // Set the direction of the motor.
        left_motor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("dual_motors/left_motor/@direction")));

        DcMotorEx right_motor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("dual_motors/right_motor"));
        right_motor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("dual_motors/right_motor/@direction")));

        motorMap = new EnumMap<FTCRobot.MotorId, DcMotorEx>(FTCRobot.MotorId.class) {{
            put(leftMotorId, left_motor);
            put(rightMotorId, right_motor);
        }};

        setZeroPowerBrakeAll();
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER); // defaults to velocity

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);
    }

    public Pair<FTCRobot.MotorId, FTCRobot.MotorId> getMotorIds() {
        return Pair.create(leftMotorId, rightMotorId);
    }

    public Pair<Integer, Integer> getTargetPositions() {
        return Pair.create(getTargetPosition(leftMotorId), getTargetPosition(rightMotorId));
    }

    public Pair<Integer, Integer> getCurrentPositions() {
        return Pair.create(getCurrentPosition(leftMotorId), getCurrentPosition(rightMotorId));
    }

    public void setModeDual(DcMotor.RunMode pRunMode) {
        setModeAll(pRunMode);
    }

    public void setTargetPositions(int pPosition) {
        setTargetPosition(leftMotorId, pPosition);
        setTargetPosition(rightMotorId, pPosition);
    }

    public void setVelocityDual(double pVelocity) {
        EnumMap<FTCRobot.MotorId, Double> velocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        velocityMap.put(leftMotorId, pVelocity);
        velocityMap.put(rightMotorId, pVelocity);
        setVelocityAll(velocityMap);
    }

    public double getVelocity() {
        return velocity;
    }

    public void stopVelocityDual() {
        stopAllZeroVelocity();
    }

    public boolean dualMotorsAreBusy() {
        return isBusy(leftMotorId) && isBusy(rightMotorId);
    }
}