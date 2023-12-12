package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;

import javax.xml.xpath.XPathExpressionException;

// Support a single motor that can be used in both Autonomous and TeleOp.
// Users of this class must ensure that the motor is in the correct
// DcMotor.RunMode. See comments in MotorCore.
public abstract class SingleMotor extends MotorCore {

    protected final FTCRobot.MotorId motorId;
    private double velocity; // This is the configured velocity from RobotConfig.xml

    public SingleMotor(HardwareMap pHardwareMap, XPathAccess pConfigXPath, FTCRobot.MotorId pMotorId) throws XPathExpressionException {
        super(pConfigXPath, "single_motor");
        motorId = pMotorId;

        String TAG = "MotorCore+" + SingleMotor.class.getSimpleName();

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the " + pMotorId + " motor");
        RobotLogCommon.c(TAG, "Motor(s) " + pConfigXPath.getRequiredText("single_motor/@model"));

        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        DcMotorEx single_motor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText("single_motor/id"));
       
        // Set the direction of the motor.
        single_motor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText("single_motor/@direction")));

        motorMap = new EnumMap<FTCRobot.MotorId, DcMotorEx>(FTCRobot.MotorId.class) {{
            put(pMotorId, single_motor);
        }};

        setZeroPowerBrake(pMotorId);
        setMode(pMotorId, DcMotor.RunMode.RUN_USING_ENCODER); // default to velocity

        velocity = pConfigXPath.getRequiredDouble("velocity");
        if (velocity <= 0.0 || velocity > 1.0)
            throw new AutonomousRobotException(TAG, "velocity out of range " + velocity);
    }

    public FTCRobot.MotorId getMotorId() {
        return motorId;
    }

    public int getTargetPosition() {
        return getTargetPosition(motorId);
    }

    public int getCurrentPosition() {
        return getCurrentPosition(motorId);
    }

    public void setMode(DcMotor.RunMode pRunMode) {
        setMode(motorId, pRunMode);
    }

    public void setTargetPosition(int pPosition) {
        setTargetPosition(motorId, pPosition);
    }

    //**TODO This really means runWithVelocity; see MotorCore
    public void setVelocity(double pVelocity) {
        setVelocity(motorId, pVelocity);
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean isBusy() {
        return isBusy(motorId);
    }
}