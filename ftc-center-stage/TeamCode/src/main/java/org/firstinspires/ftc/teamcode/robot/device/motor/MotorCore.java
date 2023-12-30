package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.EnumMap;
import java.util.Objects;

import javax.xml.xpath.XPathExpressionException;

// Base class for all DC motors. Descendants of this class may define a single
// motor, such as a shooter, or multiple motors, such as the four motors of the
// robot's drive train. All motors in a single instance of this class must be
// of the same type such as NeveRest 20.

// Users of this class must ensure that the motors are in the correct
// DcMotor.RunMode - typically RUN_TO_POSITION for Autonomous and
// RUN_WITHOUT_ENCODER for TeleOp. For RUN_TO_POSITION follow the FTC
// sample PushbotAutoDriveByEncoder_Linear and always set the run modes
// in this order: STOP_AND_RESET_ENCODER, RUN_USING_ENCODER. Then call
// setTargetPosition followed by a run mode of RUN_TO_POSITION.
public abstract class MotorCore {

    private static final String TAG = "MotorCore";

    private final double clicksPerMotorRev;
    private final double maxVelocity; // clicks per second

    protected EnumMap<FTCRobot.MotorId, DcMotorEx> motorMap;

    public MotorCore(XPathAccess pConfigXPath, String pMotorElementName) throws XPathExpressionException {
        clicksPerMotorRev = pConfigXPath.getRequiredDouble(pMotorElementName + "/@clicks_per_motor_rev");
        double motorRPM = pConfigXPath.getRequiredDouble(pMotorElementName + "/@rpm");
        maxVelocity = Math.floor((clicksPerMotorRev * motorRPM) / 60); // clicks per second
    }

    public double getClicksPerMotorRev() {
        return clicksPerMotorRev;
    }

    public DcMotor.RunMode getRunMode(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " getRunMode: motor " + pMotorId + " is not in the current configuration").getMode();
    }

    public int getCurrentPosition(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " getCurrentPosition: motor " + pMotorId + " is not in the current configuration").getCurrentPosition();
    }

    public int getTargetPosition(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " getTargetPosition: motor " + pMotorId + " is not in the current configuration").getTargetPosition();
    }

    public boolean isBusy(FTCRobot.MotorId pMotorId) {
        if (Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " isBusy: motor " + pMotorId + " is not in the current configuration").getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            throw new AutonomousRobotException(TAG, "Illegal test of isBusy((); motor " + pMotorId +
                    " has not been set to RUN_TO_POSITION; motor mode is " + getRunMode(pMotorId));

        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " isBusy: motor " + pMotorId + " is not in the current configuration").isBusy();
    }

    public void setRunMode(FTCRobot.MotorId pMotorId, DcMotor.RunMode pMode) {
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setRunMode: motor " + pMotorId + " is not in the current configuration").setMode(pMode);
    }

    public void setRunModeAll(DcMotor.RunMode pMode) {
        motorMap.forEach((k, v) -> v.setMode(pMode));
    }

    public void setZeroPowerBrake(FTCRobot.MotorId pMotorId) {
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setZeroPowerBrake: motor " + pMotorId + " is not in the current configuration").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBrakeBehavior(FTCRobot.MotorId pMotorId, DcMotor.ZeroPowerBehavior pBehavior) {
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setZeroPowerBrakeBehavior: motor " + pMotorId + " is not in the current configuration").setZeroPowerBehavior(pBehavior);
    }

    public void setZeroPowerBrakeBehaviorAll(DcMotor.ZeroPowerBehavior pBehavior) {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(pBehavior));
    }

    public void setZeroPowerBrakeAll() {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setTargetPosition(FTCRobot.MotorId pMotorId, int pTargetClicks) {
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setTargetPosition: motor " + pMotorId + " is not in the current configuration").setTargetPosition(pTargetClicks);
    }

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    public void runAtVelocity(FTCRobot.MotorId pMotorId, double pVelocity) {
        if (getRunMode(pMotorId) == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AutonomousRobotException(TAG, "Motor " + pMotorId + ": setVelocity incompatible with RUN_WITHOUT_ENCODER");

        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " runAtVelocity: motor " + pMotorId + " is not in the current configuration").setVelocity(pVelocity * maxVelocity);
    }

    public void runAtVelocityAll(EnumMap<FTCRobot.MotorId, Double> pVelocityMap) {
        pVelocityMap.forEach((k, v) ->
        {
            if (Objects.requireNonNull(motorMap.get(k),
                    TAG + " runAtVelocityAll: motor " + k + " is null in pVelocityMap").getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                throw new AutonomousRobotException(TAG, "Motor " + k + ": setVelocityAll incompatible with RUN_WITHOUT_ENCODER");

            Objects.requireNonNull(motorMap.get(k),
                    TAG + " setTargetPosition: motor " + k + " is null in pVelocityMap").setVelocity(v * maxVelocity);
        });
    }

    // For use with DcMotor.RunMode.RUN_WITHOUT_ENCODER.
    // Assumes all clipping and all final modifications to the power,
    // e.g. running at .5 power, have already been performed.
    //## Note that with either of the run modes RUN_USING_ENCODER or
    // RUN_TO_POSITION, setPower has no effect!!
    public void runAtPower(FTCRobot.MotorId pMotorId, double pPower) {
        if (getRunMode(pMotorId) != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AutonomousRobotException(TAG, "Motor " + pMotorId + ": setPower requires RUN_WITHOUT_ENCODER");

        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " runAtPower: motor " + pMotorId + " is not in the current configuration").setPower(pPower);
    }
    
    public void runAtPowerAll(EnumMap<FTCRobot.MotorId, Double> pPowerMap) {
        pPowerMap.forEach((k, v) ->
        {
            if (Objects.requireNonNull(motorMap.get(k),
                    TAG + " runAtPowerAll: motor " + k + " is null in pPowerMap").getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                throw new AutonomousRobotException(TAG, "Motor " + k + ": setPowerAll requires RUN_WITHOUT_ENCODER");

            Objects.requireNonNull(motorMap.get(k)).setPower(v);
        });
    }

    public void stopAllZeroVelocity() {
        motorMap.forEach((k, v) ->
        {
            if (v.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                throw new AutonomousRobotException(TAG, "Motor " + k + ": stopZeroVelocityAll incompatible with RUN_WITHOUT_ENCODER");
            v.setVelocity(0.0);
        });
    }

    public void stopAllZeroPower() {
        motorMap.forEach((k, v) ->
        {
            if (v.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                throw new AutonomousRobotException(TAG, "Motor " + k + ": stopAllZeroPower requires RUN_WITHOUT_ENCODER");
            v.setPower(0.0);
        });
    }

}
