package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

// For use by the action "STRAIGHT_BY" and all specializations.
public class DriveTrainPID {

    private static final String TAG = DriveTrainPID.class.getSimpleName();
    private final ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double errorSum;
    private double prevError;
    private double timeChange;
    private double proportional;
    private double integrated;
    private double derivative;

    // Proportional, Integrated, Derivative
    public DriveTrainPID(double pKp, double pKi, double pKd) {
        Kp = pKp;
        Ki = pKi;
        Kd = pKd;
        pidTimer.reset();
    }

    // Proportional and Integrated only
    public DriveTrainPID(double pKp, double pKi) {
        this(pKp, pKi, 0.0);
    }

    // Proportional only
    public DriveTrainPID(double pKp) {
        this(pKp, 0.0, 0.0);
    }

    // Perform all PID calculations and return composite value.
    //# Here's an alternative method that uses a sampling rate:
    // http://brettbeauregard.com/blog/2011/04/improving-the-beginner’s-pid-sample-time/
    // If you introduce a fixed sampling rate, watch the integral and derivative
    // calculations below. Use fractions of a second, not milliseconds.

    @SuppressLint("DefaultLocale")
    public double getPIDValue(double pError, boolean pLogVV) {
        timeChange = pidTimer.time(); // elapsed time since last call

        // If the current offset and the accumulated offset don't have the same
        // sign, restart the accumulation. See "Basics of PID Control in Robots.pptx".
        errorSum += pError; // accumulate for integral
        if (Math.signum(pError) != Math.signum(errorSum))
            errorSum = 0;

        proportional = pError * Kp;
        integrated = errorSum * timeChange * Ki;
        derivative = timeChange != 0 ? ((pError - prevError) / timeChange) * Kd : 0;
        prevError = pError;
        pidTimer.reset(); // restart elapsed timer

        if (pLogVV)
            RobotLog.vv(TAG, "p " + String.format("%.2f", proportional) +
                    ", i " + String.format("%.2f", integrated) +
                    ", d " + String.format("%.2f", derivative) +
                    ", steer " + String.format("%.2f", (proportional + integrated + derivative)));

        return proportional + integrated + derivative;
    }
}
