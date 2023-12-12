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
        return Objects.requireNonNull(motorMap.get(pMotorId)).getMode();
    }

    public int getCurrentPosition(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId)).getCurrentPosition();
    }

    public int getTargetPosition(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId)).getTargetPosition();
    }

    public boolean isBusy(FTCRobot.MotorId pMotorId) {
        if (Objects.requireNonNull(motorMap.get(pMotorId)).getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            throw new AutonomousRobotException(TAG, "Illegal test of isBusy((); motor " + pMotorId + " has not been set to RUN_TO_POSITION");

        return Objects.requireNonNull(motorMap.get(pMotorId)).isBusy();
    }

    public void setMode(FTCRobot.MotorId pMotorId, DcMotor.RunMode pMode) {
        Objects.requireNonNull(motorMap.get(pMotorId)).setMode(pMode);
    }

    public void setModeAll(DcMotor.RunMode pMode) {
        motorMap.forEach((k, v) -> v.setMode(pMode));
    }

    public void setZeroPowerBrake(FTCRobot.MotorId pMotorId) {
        Objects.requireNonNull(motorMap.get(pMotorId)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBrakeAll() {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setTargetPosition(FTCRobot.MotorId pMotorId, int pTargetClicks) {
        Objects.requireNonNull(motorMap.get(pMotorId)).setTargetPosition(pTargetClicks);
    }

    //**TODO Confusing naming: all variations of setVelocity... and setPower...
    // really mean runAt or runWith Velocity/Power

    // Assumes all clipping and all final modifications to the velocity,
    // e.g. running at .5 velocity, have already been performed.
    public void setVelocity(FTCRobot.MotorId pMotorId, double pVelocity) {
        DcMotorEx motor = motorMap.get(pMotorId);
        if (Objects.requireNonNull(motor).getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AutonomousRobotException(TAG, "Motor " + pMotorId + ": setVelocity incompatible with RUN_WITHOUT_ENCODER");

        Objects.requireNonNull(motorMap.get(pMotorId)).setVelocity(pVelocity * maxVelocity);
    }

    public void setVelocityAll(EnumMap<FTCRobot.MotorId, Double> pVelocityMap) {
        pVelocityMap.forEach((k, v) ->
        {
            if (Objects.requireNonNull(motorMap.get(k)).getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                throw new AutonomousRobotException(TAG, "Motor " + k + ": setVelocityAll incompatible with RUN_WITHOUT_ENCODER");

            Objects.requireNonNull(motorMap.get(k)).setVelocity(v * maxVelocity);
        });
    }

    // For use with DcMotor.RunMode.RUN_WITHOUT_ENCODER.
    // Assumes all clipping and all final modifications to the power,
    // e.g. running at .5 power, have already been performed.
    //## Note that with either of the run modes RUN_USING_ENCODER or
    // RUN_TO_POSITION, setPower has no effect!!
    public void setPower(FTCRobot.MotorId pMotorId, double pPower) {
        DcMotorEx motor = motorMap.get(pMotorId);
        if (Objects.requireNonNull(motor).getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new AutonomousRobotException(TAG, "Motor " + pMotorId + ": setPower requires RUN_WITHOUT_ENCODER");

        Objects.requireNonNull(motorMap.get(pMotorId)).setPower(pPower);
    }

    public void setPowerAll(EnumMap<FTCRobot.MotorId, Double> pPowerMap) {
        pPowerMap.forEach((k, v) ->
        {
            if (Objects.requireNonNull(motorMap.get(k)).getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
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
