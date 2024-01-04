package org.firstinspires.ftc.teamcode.robot.device.motor;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

// Move a single motor.
public class SingleMotorMotion {

    private static final String TAG = SingleMotorMotion.class.getSimpleName();

    public enum MotorAction {MOVE_AND_HOLD_VELOCITY, MOVE_AND_STOP}

    private final LinearOpMode linearOpMode;
    private final SingleMotor singleMotor;

    public SingleMotorMotion(LinearOpMode pLinearOpMode, SingleMotor pSingleMotor) {
        linearOpMode = pLinearOpMode;
        singleMotor = pSingleMotor;

        //## Follow the FTC sample PushbotAutoDriveByEncoder_Linear and always
        // set the run modes in this order: STOP_AND_RESET_ENCODER,
        // RUN_USING_ENCODER. Then call setTargetPosition followed by a run mode
        // of RUN_TO_POSITION. The default in this class is to run to an absolute
        // position so set STOP_AND_RESET_ENCODER only once here.
        // But the default can be overriden - see the method resetAndMoveSingleMotor
        // below.
        singleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        singleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAndMoveSingleMotor(int pTargetPosition, double pVelocity, MotorAction pMotorAction) {
        singleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveSingleMotor(pTargetPosition, pVelocity, pMotorAction);
    }

    @SuppressLint("DefaultLocale")
    public void moveSingleMotor(int pTargetPosition, double pVelocity, MotorAction pMotorAction) {
        FTCRobot.MotorId motorId = singleMotor.getMotorId();
        int motorPosition = singleMotor.getCurrentPosition();

        RobotLogCommon.d(TAG, "Motor " + motorId + " current position " + motorPosition);
        RobotLogCommon.d(TAG, "Motor " + motorId + " target position " + pTargetPosition);

        if (pTargetPosition == motorPosition)
            return; // don't bother

        // Velocity is always positive; the position determines the direction.
        double velocity = Math.abs(pVelocity);

        RobotLogCommon.d(TAG, "Move " + motorId + " to position " + pTargetPosition +
                ", at velocity " + velocity);

        singleMotor.setTargetPosition(pTargetPosition);
        singleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving.
        singleMotor.runAtVelocity(velocity);

        // Keep moving until one of the motors has reached its target position.
        try {
            while (singleMotor.isBusy()) {
                if (!linearOpMode.opModeIsActive()) {
                   RobotLog.dd(TAG, "OpMode went inactive during movement of single motor " + motorId);
                    RobotLogCommon.d(TAG, "OpMode went inactive during movement of single motor " + motorId);
                    break;
                }

                // If we're running Autonomous check the timer.
                if (FTCAuto.autonomousTimer.autoTimerIsExpired()) {
                   RobotLog.dd(TAG, "Autonomous panic stop triggered during movement of a single motor " + motorId);
                    RobotLogCommon.d(TAG, "Autonomous panic stop triggered during movement of a single motor " + motorId);
                    break;
                }

                sleep(10);
            } // while
        } finally {
            // Only stop the motor if the user has requested a stop;
            // otherwise hold the position of the motor.
            if (pMotorAction == MotorAction.MOVE_AND_STOP)
                singleMotor.runAtVelocity(0.0);

            // Log ending click counts for both motors.
            RobotLogCommon.d(TAG, "Single motor motion complete");
            RobotLogCommon.d(TAG, "Single motor " + motorId +
                    " ending position " + singleMotor.getCurrentPosition());
        }
    }
}

