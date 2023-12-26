package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotor;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;

//** To use this class just change the two lines marked with a comment
//** beginning with //!! - these are crucial.

@TeleOp(name = "IntakeMotorCalibration", group = "Test")
@Disabled
public class IntakeMotorCalibration extends TeleOpBase {

    private SingleMotor singleMotor;
    private PixelStopperServo.PixelServoState pixelServoState;

    private FTCButton outtake;
    private boolean outtakeInProgress = false;
    private FTCButton intake;
    private boolean intakeInProgress = false;
    private FTCButton reverseIntake;
    private boolean reverseIntakeInProgress = false;

    @Override
    public RobotConstants.RunType getRunType() {
        return RobotConstants.RunType.TELEOP;
    }

    @Override
    public void initialize() {

        //robot.intakeArmServo.down();

        //!! The motor you want to test must be configured in via RobotConfig.xml.
        //!! Get a reference to it from FTCRobot, e.g. singleMotor = robot.boom.
        singleMotor = robot.intakeMotor;

        outtake = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);
        intake = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
        reverseIntake = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
    }

    @Override
    public void run() {
        // The intake arm must be down before the TeleOp
        // can start.
        //if (robot.intakeArmServo != null) // will only be null in testing
        //    robot.intakeArmServo.down(); // needed only once

        // Set the initial state of the pixel stopper to HOLD
        // so that pixels can be taken in from the front.
        // This will change to RELEASE before outtake out the
        // back.
        if (robot.pixelStopperServo != null) // will only be null in testing
            robot.pixelStopperServo.hold();

        pixelServoState = PixelStopperServo.PixelServoState.HOLD;

        while (opModeIsActive()) {
            updateButtons();
            updatePlayerOne();
        }
    }

    private void updateButtons() {
        outtake.update();
        intake.update();
        reverseIntake.update();
    }

    private void updatePlayerOne() {
        updateIntake();
        updateOuttake();
        updateReverseIntake();
    }

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
                updateEncoderTelemetry();
            }
        }
    }

    private void updateReverseIntake() {
        if (reverseIntake.is(FTCButton.State.TAP) || reverseIntake.is(FTCButton.State.HELD)) {
            if (reverseIntake.is(FTCButton.State.TAP)) { // first time
                reverseIntakeInProgress = true;

                // Note that positive velocity ejects pixels to the front.
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.runAtVelocity(robot.intakeMotor.velocity);
            }
        } else {
            if (reverseIntakeInProgress) {
                reverseIntakeInProgress = false;

                robot.intakeMotor.runAtVelocity(0.0);
                updateEncoderTelemetry();
            }
        }
    }

    // Continuous outtake out the back.
    private void updateOuttake() {
        if (outtake.is(FTCButton.State.TAP) || outtake.is(FTCButton.State.HELD)) {
            if (outtake.is(FTCButton.State.TAP)) { // first time
                outtakeInProgress = true;

                // Sanity check - make sure the pixel stopper is in the release position.
                if (pixelServoState != PixelStopperServo.PixelServoState.RELEASE) {
                    robot.pixelStopperServo.release();
                    sleep(500); // give the servo time to actuate
                    pixelServoState = PixelStopperServo.PixelServoState.RELEASE;
                }

                // Note that with the stopper down negative velocity ejects
                // pixels out the back.
                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.intakeMotor.runAtVelocity(-robot.intakeMotor.velocity);
            }
        } else {
            if (outtakeInProgress) {
                outtakeInProgress = false;

                robot.intakeMotor.runAtVelocity(0.0);
                updateEncoderTelemetry();

                // Get ready for the next intake.
                robot.pixelStopperServo.hold();
                sleep(500); // give the servo time to actuate
                pixelServoState = PixelStopperServo.PixelServoState.HOLD;
            }
        }
    }

    //!! Change FTCRobot.MotorId.MOTOR_ID_NPOS to the enum value of the device under test,
    // e.g. FTCRobot.MotorId.BOOM.
    private void updateEncoderTelemetry() {
        telemetry.addData("Click count", singleMotor.getCurrentPosition(FTCRobot.MotorId.INTAKE));
        telemetry.update();
    }

}