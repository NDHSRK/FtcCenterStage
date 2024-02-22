package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Winch;
import org.firstinspires.ftc.teamcode.robot.device.servo.PixelStopperServo;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButton;
import org.firstinspires.ftc.teamcode.teleop.common.ParallelDrive;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpWithAlliance;

import java.io.IOException;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;

public class CenterStageTeleOp extends TeleOpWithAlliance {

    private static final String TAG = CenterStageTeleOp.class.getSimpleName();

    // Define buttons that return a boolean.
    private final FTCButton elevatorOnTruss;
    private final FTCButton winchUp;
    private final FTCButton winchDown;

    //&& Uncomment to calibrate the winch by small steps
    // private final FTCButton winchIncrement;
    // private final FTCButton winchDecrement;
    private int cumulativeClicks = 0;
    private static final int CLICKS_PER_WINCH_MOVEMENT = 100;

    private final FTCButton launchDrone;
    private boolean droneLaunchRequested = false;
    private final FTCToggleButton toggleSpeed;

    private final FTCButton intake;
    private boolean intakeInProgress = false;
    private final FTCButton outtake;
    private boolean outtakeInProgress = false;
    private final FTCButton goToSafe;
    private final FTCButton goToGround;
    private final FTCButton reverseIntake;
    private boolean reverseIntakeInProgress = false;
    private final FTCButton deliveryLevel1;
    private final FTCButton deliveryLevel2;

    private final FTCButton resetIntakeArm;

    // Drive train
    private double driveTrainPower;
    private double previousDriveTrainPower;
    private final double driveTrainPowerHigh;
    private final double driveTrainPowerLow;
    private final ParallelDrive parallelDrive;

    // Asynchronous
    private enum AsyncAction {MOVE_ELEVATOR_AND_WINCH, NONE}

    private AsyncAction asyncActionInProgress = AsyncAction.NONE;

    // Elevator and Winch
    private Elevator.ElevatorLevel currentElevatorLevel = Elevator.ElevatorLevel.GROUND;
    private final double elevatorVelocity;
    private CompletableFuture<Elevator.ElevatorLevel> asyncMoveElevator;

    //?? This field is assigned but never accessed.
    private Winch.WinchLevel currentWinchLevel = Winch.WinchLevel.GROUND;
    private CompletableFuture<Winch.WinchLevel> asyncMoveWinch;

    private PixelStopperServo.PixelServoState pixelServoState;

    public CenterStageTeleOp(RobotConstants.Alliance pAlliance,
                             LinearOpMode pLinearOpMode, FTCRobot pRobot,
                             @Nullable FTCAuto pAutonomous) {
        super(pAlliance, pLinearOpMode, pRobot, pAutonomous);
        RobotLogCommon.c(TAG, "Constructing CenterStageTeleOp");
        RobotLogCommon.setMostDetailedLogLevel(Objects.requireNonNull(robot.teleOpSettings, TAG + " teleOpSettings unexpectedly null").logLevel);

        driveTrainPowerHigh = robot.teleOpSettings.driveTrainPowerHigh;
        driveTrainPower = driveTrainPowerHigh;
        previousDriveTrainPower = driveTrainPower;
        driveTrainPowerLow = robot.teleOpSettings.driveTrainPowerLow;

        // The elevator can be null in testing if it has been configured out.
        if (robot.elevator != null)
            elevatorVelocity = robot.elevator.getVelocity();
        else elevatorVelocity = 0.0;

        // Gamepad 1
        elevatorOnTruss = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);
        winchUp = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
        winchDown = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        // Gamepad 1 ABXY Buttons
        launchDrone = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_Y);
        toggleSpeed = new FTCToggleButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_A);
        resetIntakeArm = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_X);

        //&& Uncomment to calibrate the winch by small steps
        // winchIncrement = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_X);
        // winchDecrement = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_B);

        // Gamepad 2
        intake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_LEFT_BUMPER);
        outtake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_RIGHT_BUMPER);
        deliveryLevel1 = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_LEFT);
        deliveryLevel2 = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);

        // Gamepad 2 ABXY Buttons
        goToSafe = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_X);
        goToGround = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_A);
        reverseIntake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_Y);

        // Start the drive train in parallel.
        parallelDrive = new ParallelDrive(linearOpMode, robot.driveTrain, driveTrainPower);
        RobotLogCommon.c(TAG, "Finished constructing CenterStageTeleOp");
    }

    @Override
    public void runTeleOp() throws Exception {
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs. From the FTC SDK documentation for opModeIsActive():
            // "If this method returns false after waitForStart() has previously
            // been called, you should break out of any loops and allow the OpMode
            // to exit at its earliest convenience."
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                return;
            }

            // The intake arm must be down before TeleOp can start.
            if (robot.intakeArmServo != null) // will only be null in testing
                robot.intakeArmServo.down(); // needed only once

            // Set the initial state of the pixel stopper to HOLD
            // so that pixels can be taken in from the front.
            // This will change to RELEASE before outtake out the
            // back.
            if (robot.pixelStopperServo != null) // will only be null in testing
                robot.pixelStopperServo.hold();

            pixelServoState = PixelStopperServo.PixelServoState.HOLD;

            if (robot.droneLauncherServo != null)
                robot.droneLauncherServo.hold();

            //## The drive train thread must be started here because
            // only now does opModeIsActive() return true.
            parallelDrive.startDriveTrain();

            while (linearOpMode.opModeIsActive()) {
                updateButtons();
                updateActions();
            }
        } finally {
            RobotLogCommon.d(TAG, "In finally() block");
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {

        // Game Controller 1
        toggleSpeed.update();
        elevatorOnTruss.update();
        winchUp.update();
        winchDown.update();

        //&& Uncomment to calibrate the winch by small steps
        // winchIncrement.update();
        // winchDecrement.update();

        launchDrone.update();
        resetIntakeArm.update();

        // Game Controller 2
        intake.update();
        outtake.update();
        reverseIntake.update();
        deliveryLevel1.update();
        deliveryLevel2.update();
        goToSafe.update();
        goToGround.update();
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() throws Exception {
        updateToggleSpeed();

        if (driveTrainPowerChanged()) {
            parallelDrive.setPower(driveTrainPower);
        }

        // If an asynchronous action is in progress do not allow any
        // actions other than those related to the drive train.
        try {
            switch (asyncActionInProgress) {
                case MOVE_ELEVATOR_AND_WINCH: {
                    if (asyncMoveElevator.isDone()) {
                        // The winch is configured in and the asynchronous movements
                        // for both the elevator and the winch have completed.
                        if (robot.winch != null && asyncMoveWinch.isDone()) {
                            currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                            currentWinchLevel = Threading.getFutureCompletion(asyncMoveWinch);
                            asyncMoveElevator = null;
                            asyncMoveWinch = null;
                            asyncActionInProgress = AsyncAction.NONE;
                            RobotLogCommon.d(TAG, "Async MOVE_ELEVATOR_AND_WINCH done");

                            if (droneLaunchRequested) {
                                droneLaunchRequested = false;
                                robot.droneLauncherServo.launch();
                                linearOpMode.sleep(500);

                                // Automatically move the elevator up to ABOVE_TRUSS
                                move_elevator_to_selected_level(Elevator.ElevatorLevel.ABOVE_TRUSS);
                            }

                            break;
                        }

                        if (robot.winch == null) {
                            // The elevator movement is complete but the winch is
                            // configured out - no need to check it.
                            currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                            asyncMoveElevator = null;
                            asyncActionInProgress = AsyncAction.NONE;
                            RobotLogCommon.d(TAG, "Async move elevator without winch done");

                            if (droneLaunchRequested) {
                                droneLaunchRequested = false;
                                robot.droneLauncherServo.launch();
                                linearOpMode.sleep(500);

                                // Automatically move the elevator up to ABOVE_TRUSS
                                move_elevator_to_selected_level(Elevator.ElevatorLevel.ABOVE_TRUSS);
                            }

                            break;
                        }
                    }

                    // The elevator and/or the winch (if configured) are still moving.
                    // Skip the updates below.
                    return;
                }
                case NONE: {
                    // continue with updates below
                    break;
                }
                default: {
                    RobotLogCommon.d(TAG, "Invalid async action " + asyncActionInProgress);
                    return; // crashing may leave the elevator in an indeterminate state
                }
            }
        } catch (TimeoutException | IOException ex) {
            // re-throw as unchecked exception
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException | TimeoutException " + eMessage);
        }

        // Game Controller 1
        updateElevatorOnTruss();
        updateWinchUp();
        updateWinchDown();

        //&& Uncomment to calibrate the winch by small steps
        // updateWinchIncrement();
        // updateWinchDecrement();

        updateLaunchDrone();
        updateResetIntakeArm();

        // Game Controller 2
        updateIntake();
        updateOuttake();
        updateReverseIntake();
        updateDeliveryLevel1();
        updateDeliveryLevel2();
        updateGoToSafe();
        updateGoToGround();
    }

    private void updateToggleSpeed() {
        if (toggleSpeed.is(FTCButton.State.TAP)) {
            FTCToggleButton.ToggleState newToggleState = toggleSpeed.toggle();
            if (newToggleState == FTCToggleButton.ToggleState.A) {
                driveTrainPower = driveTrainPowerHigh;
            } else {
                driveTrainPower = driveTrainPowerLow;
            }
        }
    }

    private boolean driveTrainPowerChanged() {
        if (driveTrainPower == previousDriveTrainPower)
            return false;

        previousDriveTrainPower = driveTrainPower;
        return true;
    }

    private void updateElevatorOnTruss() {
        if (elevatorOnTruss.is(FTCButton.State.TAP)) {
            move_elevator_to_selected_level(Elevator.ElevatorLevel.ON_TRUSS);
        }
    }


    private void updateWinchUp() {
        if (winchUp.is(FTCButton.State.TAP)) {
            if (robot.winch == null)
                return; // the winch is configured out

            if (currentElevatorLevel != Elevator.ElevatorLevel.GROUND) {
                RobotLogCommon.d(TAG, "Winch up not allowed when the elevator is at level " + currentElevatorLevel);
                return;
            }

            robot.intakeArmServo.up();
            robot.winchMotion.moveSingleMotor(robot.winch.hang, robot.winch.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            currentWinchLevel = Winch.WinchLevel.HANG;
        }
    }

    private void updateWinchDown() {
        if (winchDown.is(FTCButton.State.TAP)) {
            if (robot.winch == null)
                return; // the winch is configured out

            if (currentElevatorLevel != Elevator.ElevatorLevel.GROUND) {
                RobotLogCommon.d(TAG, "Winch down not allowed when the elevator is at level " + currentElevatorLevel);
                return;
            }

            robot.winchMotion.moveSingleMotor(robot.winch.ground, robot.winch.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_STOP);
            currentWinchLevel = Winch.WinchLevel.GROUND;
        }
    }

    private void updateLaunchDrone() {
        if (launchDrone.is(FTCButton.State.TAP)) {
            droneLaunchRequested = true;
            move_elevator_to_selected_level(Elevator.ElevatorLevel.DRONE);
        }
    }

    private void updateResetIntakeArm() {
        if (resetIntakeArm.is(FTCButton.State.TAP)) {
            robot.intakeArmServo.up();
            // give time to go up
            linearOpMode.sleep(500);
            robot.intakeArmServo.down();
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
                    linearOpMode.sleep(500); // give the servo time to actuate
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
                    linearOpMode.sleep(500); // give the servo time to actuate
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
            }
        }
    }

    private void updateDeliveryLevel1() {
        if (deliveryLevel1.is(FTCButton.State.TAP)) {
            robot.pixelStopperServo.release();
            pixelServoState = PixelStopperServo.PixelServoState.RELEASE;
            move_elevator_to_selected_level(Elevator.ElevatorLevel.LEVEL_1);
        }
    }

    private void updateDeliveryLevel2() {
        if (deliveryLevel2.is(FTCButton.State.TAP)) {
            robot.pixelStopperServo.release();
            pixelServoState = PixelStopperServo.PixelServoState.RELEASE;
            move_elevator_to_selected_level(Elevator.ElevatorLevel.LEVEL_2);
        }
    }

    //&& Uncomment to calibrate the winch by small steps
//    private void updateWinchIncrement() {
//        if (winchIncrement.is(FTCButton.State.TAP)) {
//            robot.winchMotion.moveSingleMotor(cumulativeClicks += CLICKS_PER_WINCH_MOVEMENT, robot.winch.getVelocity(),
//                    SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
//            updateWinchEncoderTelemetry();
//        }
//    }

//    private void updateWinchDecrement() {
//        if (winchDecrement.is(FTCButton.State.TAP)) {
//            robot.winchMotion.moveSingleMotor(cumulativeClicks -= CLICKS_PER_WINCH_MOVEMENT, robot.winch.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
//            updateWinchEncoderTelemetry();
//        }
//    }

    private void updateGoToSafe() {
        if (goToSafe.is(FTCButton.State.TAP)) {
            robot.pixelStopperServo.hold();
            pixelServoState = PixelStopperServo.PixelServoState.HOLD;
            move_elevator_to_selected_level(Elevator.ElevatorLevel.SAFE);
        }
    }

    private void updateGoToGround() {
        if (goToGround.is(FTCButton.State.TAP)) {
            move_elevator_to_selected_level(Elevator.ElevatorLevel.GROUND);
        }
    }

    private void move_elevator_to_selected_level(Elevator.ElevatorLevel pSelectedLevel) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Illegal: asynchronous action " + asyncActionInProgress + " is in progress during a call to move_elevator_to_selected_level()");
            return;
        }

        switch (pSelectedLevel) {
            case GROUND: {
                if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND)
                    return; // already there

                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE) {
                    RobotLogCommon.d(TAG, "Illegal attempt to move the elevator to GROUND from " + currentElevatorLevel);
                    return; // crashing may leave the elevator in an indeterminate state
                }

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to GROUND");
                robot.elevatorMotion.moveDualMotors(robot.elevator.ground, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_STOP);
                currentElevatorLevel = Elevator.ElevatorLevel.GROUND;

                // No need to run the winch motor for this small movement
                // but set its position.
                currentWinchLevel = Winch.WinchLevel.GROUND;
                break;
            }
            case SAFE: {
                if (currentElevatorLevel == Elevator.ElevatorLevel.SAFE)
                    return; // already there

                if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND) { // upward movement?
                    RobotLogCommon.d(TAG, "Moving elevator up from GROUND to SAFE");
                    robot.elevatorMotion.moveDualMotors(robot.elevator.safe, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                    currentElevatorLevel = Elevator.ElevatorLevel.SAFE;

                    // No need to run the winch motor for this small movement
                    // but set its position.
                    currentWinchLevel = Winch.WinchLevel.SAFE;
                } else { // downward movement
                    RobotLogCommon.d(TAG, "Moving elevator down to SAFE at velocity " + robot.elevator.velocity_down);
                    if (currentElevatorLevel == Elevator.ElevatorLevel.ON_TRUSS) {
                        robot.elevatorMotion.moveDualMotors(robot.elevator.safe, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                        currentElevatorLevel = Elevator.ElevatorLevel.SAFE;

                        // No need to run the winch motor for this movement
                        // but set its position.
                        currentWinchLevel = Winch.WinchLevel.ON_TRUSS;
                    }
                    else {
                        async_move_elevator(robot.elevator.safe, robot.elevator.velocity_down, Elevator.ElevatorLevel.SAFE);
                        if (robot.winch != null)
                            async_move_winch(robot.winch.safe, Winch.WinchLevel.SAFE);
                    }
                }

                break;
            }
            case LEVEL_1: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE) {
                    RobotLogCommon.d(TAG, "The elevator must be at SAFE before moving to " + pSelectedLevel);
                    return;
                }

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to LEVEL_1");
                async_move_elevator(Objects.requireNonNull(robot.elevator,
                        TAG + "  The elevator is not in the current configuration").level_1, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_1);
                if (robot.winch != null) // the winch is configured in
                    async_move_winch(robot.winch.level_1, Winch.WinchLevel.LEVEL_1);
                break;
            }
            case LEVEL_2: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE) {
                    RobotLogCommon.d(TAG, "The elevator must be at SAFE before moving to " + pSelectedLevel);
                    return;
                }

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to LEVEL_2");
                async_move_elevator(Objects.requireNonNull(robot.elevator,
                        TAG + " The elevator is not in the current configuration").level_2, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_2);
                if (robot.winch != null) // the winch is configured in
                    async_move_winch(robot.winch.level_2, Winch.WinchLevel.LEVEL_2);
                break;
            }
            case DRONE: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE) {
                    RobotLogCommon.d(TAG, "The elevator must be at SAFE before moving to " + pSelectedLevel);
                    return;
                }

                RobotLogCommon.d(TAG, "Moving elevator from SAFE to DRONE");
                async_move_elevator(Objects.requireNonNull(robot.elevator,
                        TAG + " The elevator is not in the current configuration").drone, elevatorVelocity, Elevator.ElevatorLevel.DRONE);
                if (robot.winch != null) // the winch is configured in
                    async_move_winch(robot.winch.drone, Winch.WinchLevel.DRONE);
                break;
            }
            case ON_TRUSS: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.ABOVE_TRUSS) {
                    RobotLogCommon.d(TAG, "The elevator must be at ABOVE_TRUSS before moving to ON_TRUSS");
                    return;
                }

                RobotLogCommon.d(TAG, "Moving elevator from ABOVE_TRUSS to ON_TRUSS");
                robot.elevatorMotion.moveDualMotors(Objects.requireNonNull(robot.elevator,
                        TAG + " The elevator is not in the current configuration").on_truss, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                currentElevatorLevel = Elevator.ElevatorLevel.ON_TRUSS;

                // No need to run the winch motor for this small movement
                // but set its position.
                currentWinchLevel = Winch.WinchLevel.ON_TRUSS;
                break;
            }
            case ABOVE_TRUSS: {
                if (currentElevatorLevel != Elevator.ElevatorLevel.DRONE) {
                    RobotLogCommon.d(TAG, "The elevator must be at DRONE before moving to ABOVE_TRUSS");
                    return;
                }

                RobotLogCommon.d(TAG, "Moving elevator from DRONE to ABOVE_TRUSS");
                robot.elevatorMotion.moveDualMotors(Objects.requireNonNull(robot.elevator,
                        TAG + " The elevator is not in the current configuration").above_truss, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                currentElevatorLevel = Elevator.ElevatorLevel.ABOVE_TRUSS;

                // No need to run the winch motor for this small movement
                // but set its position.
                currentWinchLevel = Winch.WinchLevel.ABOVE_TRUSS;
                break;
            }
            default: {
                RobotLogCommon.d(TAG, "Invalid elevator level " + pSelectedLevel);
                // crashing may leave the elevator in an indeterminate state
            }
        }
    }

    private void async_move_elevator(int pElevatorPosition, double pElevatorVelocity, Elevator.ElevatorLevel pElevatorLevelOnCompletion) {
         Callable<Elevator.ElevatorLevel> callableMoveElevator = () -> {
            robot.elevatorMotion.moveDualMotors(pElevatorPosition, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return pElevatorLevelOnCompletion;
        };

        asyncActionInProgress = AsyncAction.MOVE_ELEVATOR_AND_WINCH;
        RobotLogCommon.d(TAG, "Async MOVE_ELEVATOR_AND_WINCH in progress");
        asyncMoveElevator = Threading.launchAsync(callableMoveElevator);
    }

    private void async_move_winch(int pWinchPosition, Winch.WinchLevel pWinchLevelOnCompletion) {
        Callable<Winch.WinchLevel> callableMoveWinch = () -> {
            robot.winchMotion.moveSingleMotor(pWinchPosition, robot.winch.getVelocity(), SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            return pWinchLevelOnCompletion;
        };

        RobotLogCommon.d(TAG, "Async move winch in progress");
        asyncMoveWinch = Threading.launchAsync(callableMoveWinch);
    }

    private void updateWinchEncoderTelemetry() { //&& for winch calibration
        linearOpMode.telemetry.addData("current", robot.winch.getCurrentPosition(FTCRobot.MotorId.WINCH));
        linearOpMode.telemetry.update();
    }
}
