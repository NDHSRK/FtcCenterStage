package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

import java.util.List;
import java.util.Objects;
import java.util.SortedSet;

@TeleOp(name = "IntakeArmServoCalibration", group = "Test")
@Disabled
public class IntakeArmServoCalibration extends LinearOpMode {

    private static final String TAG = ServoCalibration.class.getSimpleName();
    private static final double NEUTRAL_SERVO_POSITION = 0.5;
    private static double SERVO_POSITION_CHANGE = 0.02;

    private Servo leftServo;
    private Servo rightServo;

    private FTCButton servoIncrementButton;
    private FTCButton servoDecrementButton;
    private double leftServoPosition = NEUTRAL_SERVO_POSITION;
    private double rightServoPosition = NEUTRAL_SERVO_POSITION;

    @Override
    public void runOpMode() {

        //** Hardcode tandem servo to be tested here.
        leftServo = hardwareMap.tryGet(Servo.class, "intake_arm_left");
        rightServo = hardwareMap.tryGet(Servo.class, "intake_arm_right");
        if (leftServo == null || rightServo == null) {
            telemetry.addLine("Cannot attach to servo(s)");
            telemetry.update();
            sleep(2000);

            // Log all configured servos.
            RobotLog.d(TAG, "Listing all servos");
            SortedSet<String> allServos = hardwareMap.getAllNames(Servo.class);
            allServos.forEach((name) -> RobotLog.ii(TAG, name));
            return;
        }

        telemetry.addLine("Attached to servos");
        telemetry.update();
        sleep(2000);

        servoIncrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        servoDecrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        leftServo.setPosition(NEUTRAL_SERVO_POSITION);
        rightServo.setPosition(NEUTRAL_SERVO_POSITION);
        updateTelemetry();

        telemetry.setAutoClear(true);
        telemetry.addData("-----", "controls");
        telemetry.addData("increment servo", "y");
        telemetry.addData("decrement servo", "a");
        telemetry.update();

        while (opModeIsActive()) {
            updateTelemetry();
            updateButtons();
            updateIncrement();
            updateDecrement();
        }
    }

    private void updateButtons() {
        servoIncrementButton.update();
        servoDecrementButton.update();
    }

    private void updateIncrement() {
        if (servoIncrementButton.is(FTCButton.State.TAP)) {
            leftServoPosition += SERVO_POSITION_CHANGE;
            leftServo.setPosition(leftServoPosition);
            rightServoPosition -= SERVO_POSITION_CHANGE;
            rightServo.setPosition(rightServoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    private void updateDecrement() {
        if (servoDecrementButton.is(FTCButton.State.TAP)) {
            leftServoPosition -= SERVO_POSITION_CHANGE;
            leftServo.setPosition(leftServoPosition);
            rightServoPosition += SERVO_POSITION_CHANGE;
            rightServo.setPosition(rightServoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("left servo position", leftServo.getPosition());
        telemetry.addData("right servo position ", rightServo.getPosition());
        telemetry.update();
    }
}