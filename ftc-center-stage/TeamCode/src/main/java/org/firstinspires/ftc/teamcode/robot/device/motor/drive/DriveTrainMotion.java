package org.firstinspires.ftc.teamcode.robot.device.motor.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;

public class DriveTrainMotion {

    private static final String TAG = DriveTrainMotion.class.getSimpleName();
    private final double RUN_TO_POSITION_NPOS;

    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;

    public DriveTrainMotion(LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        linearOpMode = pLinearOpMode;
        robot = pRobot;
        // Set an out-of-range position for non-dominant motors; see comments
        // below.
        RUN_TO_POSITION_NPOS = robot.driveTrain.getClicksPerInch() * 160;
    }

    // Drive the robot in a straight line while maintaining its heading.
    // See AutoDriveCore.java for an explanation of "dominant" and
    // "subordinate" motors. But an easy point to remember is that if
    // the angle of the robot's movement is evenly divisible by 90 degrees
    // then all 4 motors are dominant and all 4 run at the same velocity.

    // For all other angles, 2 motors will be dominant and 2 will be
    // subordinate.

    // This overload is the normal path.
    @SuppressLint("DefaultLocale")
    public void straight(int pTargetClicks, double pAngle, double pVelocity,
                         int pRampDownAtClicksRemaining, double pDesiredHeading) throws IOException, InterruptedException, TimeoutException {
        straight(pTargetClicks, pAngle, pVelocity,
                pRampDownAtClicksRemaining, pDesiredHeading,
                () -> false);
    }

    // This overload includes a Supplier<Boolean> that, when true, will
    // stop the run.
    @SuppressLint("DefaultLocale")
    public void straight(int pTargetClicks, double pAngle, double pVelocity,
                         int pRampDownAtClicksRemaining, double pDesiredHeading,
                         Supplier<Boolean> pCutShort) {

        int targetClicks = Math.abs(pTargetClicks); // ensure consistency
        int rampDownAtClicksRemaining = Math.abs(pRampDownAtClicksRemaining); // ensure consistency

        RobotLogCommon.d(TAG, "Start straight drive of " + targetClicks + " clicks at angle " +
                String.format("%.2f", pAngle) +
                ", ramp down at abs " + rampDownAtClicksRemaining +
                " clicks remaining, desired heading " + String.format("%.2f", pDesiredHeading));

        RobotLogCommon.d(TAG, "Straight drive velocity " + pVelocity);
        AutoDrive adc = new AutoDrive(pAngle, pVelocity, DriveTrainConstants.MINIMUM_DOMINANT_MOTOR_VELOCITY);
        EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> allDriveMotors = adc.getDriveMotorData();

        // Prepare all motors.
        //## Follow the FTC sample PushbotAutoDriveByEncoder_Linear and always
        // set the run modes in this order: STOP_AND_RESET_ENCODER,
        // RUN_USING_ENCODER. Then, if running to a position, call
        // setTargetPosition followed by a run mode of RUN_TO_POSITION.
        EnumMap<FTCRobot.MotorId, Double> velocityMap = new EnumMap<>(FTCRobot.MotorId.class);
        Objects.requireNonNull(robot.driveTrain).setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLogCommon.d(TAG, "Initial motor settings");

        FTCRobot.MotorId dominantMotorId = FTCRobot.MotorId.MOTOR_ID_NPOS;
        for (Map.Entry<FTCRobot.MotorId, AutoDrive.DriveMotorData> oneMotorEntry : allDriveMotors.entrySet()) {

            AutoDrive.DriveMotorData oneMotor = oneMotorEntry.getValue();
            FTCRobot.MotorId motorId = oneMotorEntry.getKey();

            // Set all motors to use RUN_TO_POSITION.
            if (oneMotor.motorRank == DriveTrainConstants.MotorRank.DOMINANT) {
                robot.driveTrain.setTargetPosition(motorId, targetClicks * oneMotor.directionSignum);
                robot.driveTrain.setMode(motorId, DcMotor.RunMode.RUN_TO_POSITION);
                dominantMotorId = motorId; // can be any one
            } else {
                // We use RUN_TO_POSITION with non-dominant motors also for consistent
                // motor control. But we set a position that's out of range for the field
                // to make sure that only the dominant motors will ever reach their
                // target position. Note that for a 45-degree angle the non-dominant
                // motors receive zero velocity. Below we only check the dominant motors
                // for isBusy().
                robot.driveTrain.setTargetPosition(motorId, (int) (RUN_TO_POSITION_NPOS * oneMotor.directionSignum));
                robot.driveTrain.setMode(motorId, DcMotor.RunMode.RUN_TO_POSITION);
            }

            velocityMap.put(motorId, oneMotor.initialVelocity);
            RobotLogCommon.d(TAG, "Motor " + motorId + ", mode " + robot.driveTrain.getRunMode(motorId) +
                    ", " + oneMotor.motorRank +
                    ", clicks " + targetClicks * oneMotor.directionSignum +
                    ", initial velocity " + String.format("%.2f", oneMotor.initialVelocity));
        }

        // Sanity check
        if (dominantMotorId == FTCRobot.MotorId.MOTOR_ID_NPOS)
            throw new AutonomousRobotException(TAG, "Failed to find dominant motor");

        StraightDriveRampdown straightDriveRampdown = new StraightDriveRampdown(robot, rampDownAtClicksRemaining, allDriveMotors, dominantMotorId);

        // Start moving.
        robot.driveTrain.runAtVelocityAll(velocityMap);
        DriveTrainPID driveTrainPID = new DriveTrainPID(DriveTrainConstants.P_DRIVE_COEFF);

        // Keep moving until one of the dominant motors has reached its target
        // position.
        try {
            int logVV = -1;
            double currentHeading;
            double steer;
            int dominantMotorClickCount;
            int remainingClickCount;
            double rampDownFactor = 1.0; // start with no ramp-down
            while (true) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLogCommon.d(TAG, "OpMode went inactive during straight line run");
                    break;
                }

                if (pCutShort.get()) {
                    RobotLogCommon.d(TAG, "Stop straight run at request of caller");
                    break;
                }

                // When any one of the motors goes not busy, run to position is
                // complete.
                if (allDriveMotors.entrySet().stream()
                        .filter(e -> e.getValue().motorRank == DriveTrainConstants.MotorRank.DOMINANT)
                        .anyMatch(e -> !robot.driveTrain.isBusy(e.getKey())))
                    break; // exit while loop

                //**TODO                if (turnLogVV % RobotConstants.VV_LOGGING_SAMPLING_FREQUENCY == 0)
                currentHeading = Objects.requireNonNull(robot.imuDirect).getIMUHeading();
                steer = applyConstantHeadingPID(pDesiredHeading, currentHeading, pAngle,
                        allDriveMotors, driveTrainPID, rampDownFactor);

                // Make sure the next call to straightDriveRampdown does not
                // counteract the PID. The PID method only changes motor velocity
                // if abs(steer) >= MINIMUM_DRIVE_POWER_STEP, but if the velocity
                // is changed, make sure to pass the PID factor "steer" to
                // straightDriveRampdown so that it can be incorporated into the
                // new ramped-down velocity.

                // Check for requested velocity ramp-down.
                // Get the click count of one of the dominant motors.
                dominantMotorClickCount = Math.abs(robot.driveTrain.getCurrentPosition(dominantMotorId));
                remainingClickCount = targetClicks - dominantMotorClickCount;
                if (rampDownAtClicksRemaining != 0.0 &&
                        dominantMotorClickCount < targetClicks &&
                        remainingClickCount <= rampDownAtClicksRemaining)
                    rampDownFactor = straightDriveRampdown.rampDown(remainingClickCount, pAngle, steer);
            } // while
        } finally {
            robot.driveTrain.stopAllZeroVelocity();

            // Log ending click counts for all dominant motors.
            RobotLogCommon.d(TAG, "Straight line drive complete");
            allDriveMotors.forEach((motorId, motorData) -> {
                if (motorData.motorRank == DriveTrainConstants.MotorRank.DOMINANT)
                    RobotLogCommon.d(TAG, "Dominant motor " + motorId +
                            " ending encoder " + robot.driveTrain.getCurrentPosition(motorId));
            });
        }
    }

    //**TODO Why can't you use RUN_USING_ENCODER and velocity here? If you do this
    // then RobotAction.xml and XPath matching will also have to change.
    // See the sample RobotAutoDriveByGyro_Linear, which uses power - try there first.

    // Executes a turn.
    // Takes into account the fact that the desired heading of the robot before the turn may not be the same
    // as the current heading. So this method either increases or diminishes the turn to make up the gap.
    // Returns the desired heading after the turn.
    // Honors the pStartRampDown parameter, which tells this method at what point (in degrees) to start
    // ramping down the power as a way of avoiding overturning.
    //?? Import stall detection from Skystone Summer 2020 if needed.

    // This overload is the normal path.
    public double turn(double pDesiredHeadingBeforeTurn, double pCurrentHeading, double pTurnDegrees,
                       double pPower, double pStartRampDownDegrees, DriveTrainConstants.TurnNormalization pTurnNormalization) throws IOException, InterruptedException, TimeoutException {
        return turn(pDesiredHeadingBeforeTurn, pCurrentHeading, pTurnDegrees,
                pPower, pStartRampDownDegrees, pTurnNormalization,
                () -> false);
    }

    // This overload includes a Supplier<Boolean> that, when true, will
    // stop the run.
    @SuppressLint("DefaultLocale")
    public double turn(double pDesiredHeadingBeforeTurn, double pCurrentHeading, double pTurnDegrees, double pPower, double pStartRampDownDegrees, DriveTrainConstants.TurnNormalization pTurnNormalization,
                       Supplier<Boolean> pCutShort) throws IOException, InterruptedException, TimeoutException {

        int turnLogVV = -1;
        double startRampDown = Math.abs(pStartRampDownDegrees); // abs for consistency

        RobotLogCommon.d(TAG, "Start turn, desired heading before turn " + String.format("%.2f", pDesiredHeadingBeforeTurn) +
                ", current heading before turn " + String.format("%.2f", pCurrentHeading) +
                ", requested turn angle " + String.format("%.2f", pTurnDegrees) +
                ", power " + String.format("%.2f", pPower) +
                ", start ramp down at " + String.format("%.2f", startRampDown) + " degrees remaining, turn normalization " + pTurnNormalization);

        Objects.requireNonNull(robot.driveTrain).setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // the IMU controls the turn

        double currentHeading = pCurrentHeading;
        Headings.TurnData turnData = Headings.getActualTurn(pDesiredHeadingBeforeTurn, currentHeading, pTurnDegrees, pTurnNormalization);
        RobotLogCommon.d(TAG, "Start turn: actual angle to turn " + String.format("%.2f", turnData.actualTurn) +
                ", desired heading after turn " + String.format("%.2f", turnData.desiredHeadingAfterTurn));

        // Sanity check: nothing to do for a turn of 0 degrees.
        if (turnData.actualTurn == 0.0)
            return pDesiredHeadingBeforeTurn;

        // A CW turn requires positive power to the left side motors and
        // negative power to the right side motors.
        double power = Math.abs(pPower); // start with positive power
        EnumMap<FTCRobot.MotorId, Double> powerMap = new EnumMap<>(FTCRobot.MotorId.class);
        powerMap.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, power * Math.signum(turnData.actualTurn) * -1);
        powerMap.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, power * Math.signum(turnData.actualTurn));
        powerMap.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, power * Math.signum(turnData.actualTurn) * -1);
        powerMap.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, power * Math.signum(turnData.actualTurn));

        // Set up for ramp down.
        TurnRampdown turnRampdown = new TurnRampdown(robot, startRampDown, powerMap);

        // Start turn.
        robot.driveTrain.runAtPowerAll(powerMap);

        try {
            // Keep looping while we are still active and have not yet reached the desired heading.
            double previousCurrentHeading;
            double degreeDifference;
            double degreesTurned = 0;
            double remainingAngle;
            boolean turnComplete = false;
            while (!turnComplete) {
                if (!linearOpMode.opModeIsActive()) {
                    RobotLogCommon.d(TAG, "OpMode went inactive during turn");
                    break;
                }

                if (pCutShort.get()) {
                    RobotLogCommon.d(TAG, "Stop turn at request of caller");
                    break;
                }

                // Verified calculation of remaining angle taken from HeadingUtils::simulateTurn
                // in Visual Studio project AngleCalculator.

                // If the robot has reached the turn window, e.g. 2 degrees, stop here.
                // Otherwise keep turning.
                turnLogVV++;
                previousCurrentHeading = currentHeading;
                currentHeading = robot.imuDirect.getIMUHeading();
                degreeDifference = Headings.normalize(currentHeading - previousCurrentHeading, -180, 180);
                degreesTurned += degreeDifference;
                remainingAngle = turnData.actualTurn - degreesTurned;

                if (turnLogVV % RobotConstants.VV_LOGGING_SAMPLING_FREQUENCY == 0)
                  RobotLogCommon.vv(TAG, "Current heading " + currentHeading + " remaining angle " + remainingAngle);

                // Make sure that the sign of the remaining angle is the same as that of
                // the original turn.
                if (Math.signum(remainingAngle) != Math.signum(turnData.actualTurn)) {
                    RobotLogCommon.d(TAG, "Overshot turn: remaining angle " + String.format("%.2f", remainingAngle));
                    turnComplete = true;
                    continue;
                }

                // Now the remaining angle will always be positive.
                remainingAngle = Math.abs(remainingAngle);
                if (remainingAngle <= DriveTrainConstants.TURN_THRESHOLD_DEGREES) {
                    RobotLogCommon.d(TAG, "Reached turn threshold; turn complete");
                    RobotLogCommon.d(TAG, "Current heading at turn complete " + String.format("%.2f", currentHeading));
                    turnComplete = true;
                    continue;
                }

                // Check for a ramp-down in power as the robot approaches its target.
                if (startRampDown != 0.0 && remainingAngle <= startRampDown)
                    turnRampdown.rampDown(remainingAngle);
            }

            return turnData.desiredHeadingAfterTurn;
        } // try

        // In case of any unforeseen conditions always stop the motors.
        finally {
            robot.driveTrain.stopAllZeroPower();
            RobotLogCommon.d(TAG, "IMU heading after turn " + String.format("%.2f", robot.imuDirect.getIMUHeading()));
        }
    }

    // Applies a PID and adjusts the velocity of each motor.
    // Running continuously results in frequent small updates to motor velocity.
    // It's better to adjust the velocity only when the absolute value of the
    // output of the PID (the "steer" value) is equal to or greater than the
    // minimum power step, e.g. .05. The parameter pRampDownFactor is included
    // so that the PID can be applied as the robot's velocity ramps down
    // according to the optional value in RobotAction.xml.

    // Apply PID to a straight line movement of the robot where the
    // heading should remain constant: forward, back, and all variations
    // of strafing.
    @SuppressLint("DefaultLocale")
    private double applyConstantHeadingPID(double pDesiredHeading, double pCurrentHeading, double pAngle,
                                           EnumMap<FTCRobot.MotorId, AutoDrive.DriveMotorData> pCurrentMotorData,
                                           DriveTrainPID pPID, double pRampDownFactor) {

        double error = DEGREES.normalize(pDesiredHeading - pCurrentHeading);

        //**TODO Log every 5th iteration ... need boolean parameter
        RobotLogCommon.vv(TAG, "IMU " + String.format("%.2f", pCurrentHeading) +
                ", error " + String.format("%.2f", error));
        double steer = pPID.getPIDValue(error);

        if (Math.abs(steer) < DriveTrainConstants.MINIMUM_DRIVE_POWER_STEP)
            return steer; // velocity increment too small, skip

        EnumMap<FTCRobot.MotorId, Double> newVelocityMap = MotionUtils.updateDriveTrainVelocity(pCurrentMotorData, pAngle, steer, pRampDownFactor);
        Objects.requireNonNull(robot.driveTrain).runAtVelocityAll(newVelocityMap);

        //**TODO Log every 5th iteration ... need boolean parameter
        RobotLogCommon.vv(TAG, "Straight velocity lf " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.LEFT_FRONT_DRIVE)) +
                ", rf " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.RIGHT_FRONT_DRIVE)) +
                ", lb " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.LEFT_BACK_DRIVE)) +
                ", rb " + String.format("%.2f", newVelocityMap.get(FTCRobot.MotorId.RIGHT_BACK_DRIVE)));

        return steer;
    }

}
