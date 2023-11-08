package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.device.motor.MotionUtils;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrain;

public class DriveStick2 {

    private final LinearOpMode linear;
    private final DriveTrain driveTrain;

    public DriveStick2(LinearOpMode pLinear, DriveTrain pDriveTrain) {
        linear = pLinear;
        driveTrain = pDriveTrain;

        // Override the default run mode for driving by the game
        // controller.
        driveTrain.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateDrive(double pPowerFactor) {

        //## The following comes from the sample BasicOmniOpMode_Linear.
        //## The only difference is that we multiply the power by a
        //## fractional value - pPowerFactor.
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -linear.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  linear.gamepad1.left_stick_x;
        double yaw     =  linear.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (axial + lateral + yaw) * pPowerFactor;
        double rightFrontPower = (axial - lateral - yaw) * pPowerFactor;
        double leftBackPower   = (axial - lateral + yaw) * pPowerFactor;
        double rightBackPower  = (axial + lateral - yaw) * pPowerFactor;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // End excerpt from the sample BasicOmniOpMode_Linear.

        driveTrain.driveAllByPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

}