package org.firstinspires.ftc.teamcode.robot.device.motor;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

// Move two motors in tandem.
public class DualMotorMotion {

    private static final String TAG = DualMotorMotion.class.getSimpleName();

    public enum DualMotorAction {MOVE_AND_HOLD_VELOCITY, MOVE_AND_STOP}

    private final LinearOpMode linearOpMode;
    private final DualMotors dualMotors;
    private boolean abnormalTermination = false;

    public DualMotorMotion(LinearOpMode pLinearOpMode, DualMotors pDualMotors) {
        linearOpMode = pLinearOpMode;
        dualMotors = pDualMotors;

        // Set the run mode for both motors.
        //## Follow the FTC sample PushbotAutoDriveByEncoder_Linear and always
        // set the run modes in this order: STOP_AND_RESET_ENCODER,
        // RUN_USING_ENCODER. Then call setTargetPosition followed by a run mode
        // of RUN_TO_POSITION. In this class we only set STOP_AND_RESET_ENCODER
        // once because we'll always run to an absolute position.
        dualMotors.setModeDual(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dualMotors.setModeDual(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAndMoveDualMotors(int pTargetPosition, double pVelocity, DualMotorAction pMotorAction) {
        dualMotors.setModeDual(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveDualMotors(pTargetPosition, pVelocity, pMotorAction);
    }

    // Note: all target positions are absolute.
    @SuppressLint("DefaultLocale")
    public void moveDualMotors(int pTargetPosition, double pVelocity, DualMotorAction pDualMotorAction) {
        abnormalTermination = false;

        Pair<FTCRobot.MotorId, FTCRobot.MotorId> motorIds = dualMotors.getMotorIds();
        Pair<Integer, Integer> currentMotorPositions = dualMotors.getCurrentPositions(); // first = left motor; second = right motor    

        RobotLogCommon.d(TAG, "Left motor " + motorIds.first + " current position " + currentMotorPositions.first);
        RobotLogCommon.d(TAG, "Left motor " + motorIds.first + " target position " + pTargetPosition);
        RobotLogCommon.d(TAG, "Right motor " + motorIds.second + " current position " + currentMotorPositions.second);
        RobotLogCommon.d(TAG, "Right motor " + motorIds.second + " target position " + pTargetPosition);

        double velocity = Math.abs(pVelocity); // velocity is always positive; position determines direction
        RobotLogCommon.d(TAG, "Move dual motors to position " + pTargetPosition + ", at velocity " + velocity);

        dualMotors.setTargetPositions(pTargetPosition);
        dualMotors.setModeDual(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving.
        dualMotors.runDualMotorsAtVelocity(velocity);

        // Keep moving until one of the motors has reached its target position.
        try {
            while (dualMotors.dualMotorsAreBusy()) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLog.ee(TAG, "OpMode went inactive during movement of dual motors " + motorIds.first + " and " + motorIds.second);
                    RobotLogCommon.d(TAG, "OpMode went inactive during movement of dual motors " + motorIds.first + " and " + motorIds.second);
                    break;
                }

                // If we're running Autonomous check the timer.
                if (FTCAuto.autonomousTimer != null && FTCAuto.autonomousTimer.autoTimerIsExpired()) {
                    RobotLog.dd(TAG, "Autonomous panic stop triggered during movement of dual motors " + motorIds.first + " and " + motorIds.second);
                    RobotLogCommon.d(TAG, "Autonomous panic stop triggered during movement of dual motors " + motorIds.first + " and " + motorIds.second);
                    // Do not set the abnormalTermination flag - we want the code in finally() to run.
                    break;
                }
            }
        } catch (Exception ex) {
            abnormalTermination = true;
            throw ex;
        } finally {
            if (!linearOpMode.opModeIsActive() || abnormalTermination) {
              RobotLog.ee(TAG, "Abnormal termination");
              return;
            }

            // Only stop the motors if the user has requested a stop; otherwise hold their position.
            if (pDualMotorAction == DualMotorAction.MOVE_AND_STOP)
                dualMotors.stopVelocityDual();

            // Log ending click counts for both motors.
            RobotLogCommon.d(TAG, "Dual motor motion complete");
            Pair<Integer, Integer> dualMotorPositions = dualMotors.getCurrentPositions();
            RobotLogCommon.d(TAG, "Motor " + motorIds.first +
                    " ending position " + dualMotorPositions.first);
            RobotLogCommon.d(TAG, "Motor " + motorIds.second +
                    " ending position " + dualMotorPositions.second);
        }
    }

    // Move both motors downward until they each trip their respective magnetic switch.
    public void moveDualMotorsDownToMagneticLimit(TouchSensor pLeftSwitch, TouchSensor pRightSwitch, double pVelocity) {
        dualMotors.setModeDual(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pair<FTCRobot.MotorId, FTCRobot.MotorId> motorIds = dualMotors.getMotorIds();
        RobotLogCommon.d(TAG, "Moving dual motors " + motorIds.first + ", " +
                motorIds.second + " down to their magnetic limit");

        try {
            boolean reachedLeftLimit = false;
            boolean reachedRightLimit = false;
            double velocity = -Math.abs(pVelocity); // velocity is always negative
            dualMotors.runDualMotorsAtVelocity(velocity); // start moving

            while (!reachedLeftLimit || !reachedRightLimit) {
                if (!reachedLeftLimit && pLeftSwitch.isPressed()) {
                    reachedLeftLimit = true;
                    dualMotors.runAtVelocity(motorIds.first, 0.0);
                }

                if (!reachedRightLimit && pRightSwitch.isPressed()) {
                    reachedRightLimit = true;
                    dualMotors.runAtVelocity(motorIds.second, 0.0);
                }

                sleep(10);
            }
        } finally {
            dualMotors.stopVelocityDual(); // for safety in case of an exception

            RobotLogCommon.d(TAG, "Dual motor motion down to magnetic limit complete");
            Pair<Integer, Integer> dualMotorPositions = dualMotors.getCurrentPositions();
            RobotLogCommon.d(TAG, "Motor " + motorIds.first +
                    " ending position " + dualMotorPositions.first);
            RobotLogCommon.d(TAG, "Motor " + motorIds.second +
                    " ending position " + dualMotorPositions.second);

            // Reset the zero point of the encoders.
            dualMotors.setModeDual(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}

