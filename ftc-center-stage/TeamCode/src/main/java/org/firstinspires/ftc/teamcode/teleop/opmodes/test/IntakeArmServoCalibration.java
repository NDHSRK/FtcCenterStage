package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

import java.util.List;
import java.util.Objects;
import java.util.SortedSet;

// Inherit from TeleOpBase because we need FTCRobot to
// set up several devices.
@TeleOp(name = "IntakeArmServoCalibration", group = "Test")
//@Disabled
public class IntakeArmServoCalibration extends TeleOpBase {

    private static final String TAG = ServoCalibration.class.getSimpleName();
    private static final double NEUTRAL_SERVO_POSITION = 0.5;
    private static double SERVO_POSITION_CHANGE = 0.02;

    private FTCButton intake;
    private boolean intakeInProgress = false;
    private FTCButton servoIncrementButton;
    private FTCButton servoDecrementButton;
    private double leftServoPosition = NEUTRAL_SERVO_POSITION;
    private double rightServoPosition = NEUTRAL_SERVO_POSITION;
    private PixelStopperServo.PixelServoState pixelServoState;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {
        servoIncrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        servoDecrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        intake = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);

        telemetry.setAutoClear(true);
        telemetry.addData("-----", "controls");
        telemetry.addData("increment servo", "Y");
        telemetry.addData("decrement servo", "A");
        telemetry.addData("intake", "hold LEFT BUMPER");
        telemetry.update();
    }

    @Override
    public void run() {
        // The intake arm must be down before TeleOp can start.
        if (robot.intakeArmServo != null) // will only be null in testing
            robot.intakeArmServo.down(); // needed only once

        // Set the initial state of the pixel stopper to HOLD
        // so that pixels can be taken in from the front.
        if (robot.pixelStopperServo != null) // will only be null in testing
            robot.pixelStopperServo.hold();

        pixelServoState = PixelStopperServo.PixelServoState.HOLD;

        robot.intakeArmServo.leftServo.setPosition(NEUTRAL_SERVO_POSITION);
        robot.intakeArmServo.rightServo.setPosition(NEUTRAL_SERVO_POSITION);
        updateTelemetry();

        while (opModeIsActive()) {
            updateTelemetry();
            updateButtons();
            updateIncrement();
            updateDecrement();
            updateIntake();
        }
    }

    private void updateButtons() {
        servoIncrementButton.update();
        servoDecrementButton.update();
        intake.update();
    }

    private void updateIncrement() {
        if (servoIncrementButton.is(FTCButton.State.TAP)) {
            leftServoPosition += SERVO_POSITION_CHANGE;
            robot.intakeArmServo.leftServo.setPosition(leftServoPosition);
            rightServoPosition -= SERVO_POSITION_CHANGE;
            robot.intakeArmServo.rightServo.setPosition(rightServoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    private void updateDecrement() {
        if (servoDecrementButton.is(FTCButton.State.TAP)) {
            leftServoPosition -= SERVO_POSITION_CHANGE;
            robot.intakeArmServo.leftServo.setPosition(leftServoPosition);
            rightServoPosition += SERVO_POSITION_CHANGE;
            robot.intakeArmServo.rightServo.setPosition(rightServoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    // Continuous intake.
    private void updateIntake() {
        if (intake.is(FTCButton.State.TAP) || intake.is(FTCButton.State.HELD)) {
            if (intake.is(FTCButton.State.TAP)) { // first time
                intakeInProgress = true;

                // Sanity check - make sure the pixel stopper is in the hold position.
                if (pixelServoState != PixelStopperServo.PixelServoState.HOLD) {
                    robot.pixelStopperServo.hold();
                    sleep(500); // give the servo time to actuate
                    pixelServoState = PixelStopperServo.PixelServoState.HOLD;
                }

                // Note that negative velocity pulls pixels in from the front.
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.runAtVelocity(-robot.intakeMotor.velocity);
            }
        } else {
            if (intakeInProgress) {
                intakeInProgress = false;
                robot.intakeMotor.runAtVelocity(0.0);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("left servo position", robot.intakeArmServo.leftServo.getPosition());
        telemetry.addData("right servo position ", robot.intakeArmServo.rightServo.getPosition());
        telemetry.update();
    }

}