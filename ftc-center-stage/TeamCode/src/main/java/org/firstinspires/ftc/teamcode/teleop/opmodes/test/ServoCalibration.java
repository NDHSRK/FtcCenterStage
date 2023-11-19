package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

import java.util.Objects;

@TeleOp(name = "ServoCalibration", group = "Test")
@Disabled
public class ServoCalibration extends TeleOpBase {

    private static final String TAG = ServoCalibration.class.getSimpleName();

    private Servo servo;

    private FTCButton servoIncrementButton;
    private FTCButton servoDecrementButton;
    private double servoPosition = 0.0; // usually .5
    private static double servoPositionChange = 0.1; // intake arm holder
            // 0.1; // hardcode for carrier
            // 0.01; // hardcode for the rake lifter
            // 0.05 hardcode for the claw

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {

        //** Hardcode servo to be tested here.
        servo = hardwareMap.get(Servo.class, "drone_launcher");

        servo.setPosition(servoPosition);
        sleep(1000);

        servoIncrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        servoDecrementButton = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);

        telemetry.setAutoClear(true);
        telemetry.addData("-----", "controls");
        telemetry.addData("increment servo", "y");
        telemetry.addData("decrement servo", "a");
        telemetry.update();
    }

    @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
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
            servoPosition += servoPositionChange;
            servo.setPosition(servoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    private void updateDecrement() {
        if (servoDecrementButton.is(FTCButton.State.TAP)) {
            servoPosition -= servoPositionChange;
            servo.setPosition(servoPosition);
            sleep(1000);
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("servo position", servoPosition);
        telemetry.update();
    }
}