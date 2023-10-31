package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.motor.Boom;
import org.firstinspires.ftc.teamcode.robot.device.motor.DualMotorMotion;
import org.firstinspires.ftc.teamcode.robot.device.motor.Elevator;
import org.firstinspires.ftc.teamcode.robot.device.motor.SingleMotorMotion;
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
    private final FTCButton hangUp;
    private final FTCButton hangDown;
    private final FTCToggleButton toggleSpeed;
    private final FTCButton intake;
    private final FTCButton outtake;
    private final FTCButton deliveryLevel1;
    private final FTCButton deliveryLevel2;
    //**TODO 10/31/23 disable for now private final FTCButton deliveryLevel3;
    private final FTCButton goToSafe;
    private final FTCButton goToGround;

    // Drive train
    private double driveTrainVelocity;
    private double previousDriveTrainVelocity;
    private final double driveTrainVelocityHigh;
    private final double driveTrainVelocityLow;
    private final ParallelDrive parallelDrive;

    // Asynchronous
    private enum AsyncAction {MOVE_ELEVATOR_UP_AND_BOOM_OUT, MOVE_ELEVATOR_DOWN_AND_BOOM_IN, NONE}
    private AsyncAction asyncActionInProgress = AsyncAction.NONE;

    // Elevator
    private Elevator.ElevatorLevel currentElevatorLevel = Elevator.ElevatorLevel.GROUND;
    private final DualMotorMotion elevatorMotion;
    private final double elevatorVelocity;
    private CompletableFuture<Elevator.ElevatorLevel> asyncMoveElevator;

    // Boom
    private Boom.BoomLevel currentBoomLevel = Boom.BoomLevel.REST;
    private final SingleMotorMotion boomMotion;
    private final double boomVelocity;
    private CompletableFuture<Boom.BoomLevel> asyncMoveBoom;

    public CenterStageTeleOp(RobotConstants.Alliance pAlliance,
                             LinearOpMode pLinearOpMode, FTCRobot pRobot,
                             @Nullable FTCAuto pAutonomous) {
        super(pAlliance, pLinearOpMode, pRobot, pAutonomous);
        RobotLogCommon.c(TAG, "Constructing CenterStageTeleOp");
        RobotLogCommon.setMostDetailedLogLevel(Objects.requireNonNull(robot.teleOpSettings, "robot.teleOpSettings unexpectedly null").logLevel);

        driveTrainVelocityHigh = robot.teleOpSettings.driveTrainVelocityHigh;
        driveTrainVelocity = driveTrainVelocityHigh;
        previousDriveTrainVelocity = driveTrainVelocity;
        driveTrainVelocityLow = robot.teleOpSettings.driveTrainVelocityLow;

        elevatorVelocity = Objects.requireNonNull(robot.elevator).getVelocity();
        boomVelocity = Objects.requireNonNull(robot.boom).getVelocity();

        elevatorMotion = new DualMotorMotion(linearOpMode, robot.elevator);
        boomMotion = new SingleMotorMotion(linearOpMode, robot.boom);

        // Gamepad 1
        hangDown = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
        hangUp = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);
        toggleSpeed = new FTCToggleButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_1_A);

        // Gamepad 2
        // Bumpers
        outtake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_RIGHT_BUMPER);
        intake = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_LEFT_BUMPER);

        // ABXY Buttons
        goToSafe = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_X);
        goToGround = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_A);

        // D-Pad
        deliveryLevel1 = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_LEFT);
        deliveryLevel2 = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);
        //**TODO 10/31/23 disable for now deliveryLevel3 = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_RIGHT);
        // Start the drive train in parallel.
        parallelDrive = new ParallelDrive(linearOpMode, robot.driveTrain, driveTrainVelocity);
        RobotLogCommon.c(TAG, "Finished constructing CenterStageTeleOp");
    }

    @Override
    public void runTeleOp() throws InterruptedException {
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

            // The intake arm holder must be down before the boom
            // can move.
            robot.intakeArmHolderServo.hold();

            //## The drive train thread must be started here because
            // only now does opModeIsActive() return true.
            parallelDrive.startDriveTrain();

            while (linearOpMode.opModeIsActive()) {
                updateButtons();
                updateActions();
            }
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, linearOpMode);
        }
    }

    // Update the state of the active buttons. This method should be
    // called once per cycle.
    private void updateButtons() {

        // Game Controller 1
        toggleSpeed.update();
        hangUp.update();
        hangDown.update();

        // Game Controller 2
        intake.update();
        outtake.update();
        deliveryLevel1.update();
        deliveryLevel2.update();
        //**TODO 10/31/23 disable for now deliveryLevel3.update();
        goToSafe.update();
        goToGround.update();
    }

    // Execute the actions controlled by Player 1 and Player 2.
    // This method should be called once per cycle.
    private void updateActions() throws Exception {
        updateToggleSpeed();

        if (velocityChanged())
            parallelDrive.setVelocity(driveTrainVelocity);

        // If an asynchronous action is in progress do not allow any
        // actions other than those related to the drive train.
        try {
            switch (asyncActionInProgress) {
                case MOVE_ELEVATOR_UP_AND_BOOM_OUT: {
                    if (asyncMoveElevator.isDone() && asyncMoveBoom.isDone()) {
                        currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                        currentBoomLevel = Threading.getFutureCompletion(asyncMoveBoom);

                        asyncMoveElevator = null;
                        asyncMoveBoom = null;
                        asyncActionInProgress = AsyncAction.NONE;
                        RobotLogCommon.v(TAG, "Async MOVE_ELEVATOR_UP_AND_BOOM_OUT done");
                    } else // the elevator and/or the boom are still moving
                        return; // skip the updates below
                    break;
                }
                case MOVE_ELEVATOR_DOWN_AND_BOOM_IN: {
                    if (asyncMoveElevator.isDone() && asyncMoveBoom.isDone()) {
                        currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                        currentBoomLevel = Threading.getFutureCompletion(asyncMoveBoom);

                        // The elevator is at CLEAR; move it to SAFE.
                        elevatorMotion.moveDualMotors(robot.elevator.safe, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                        currentElevatorLevel = Elevator.ElevatorLevel.SAFE;

                        asyncMoveElevator = null;
                        asyncMoveBoom = null;
                        asyncActionInProgress = AsyncAction.NONE;
                        RobotLogCommon.v(TAG, "Async MOVE_ELEVATOR_DOWN_AND_BOOM_IN done");
                    } else // the elevator and/or the arm are still moving
                        return; // skip the updates below
                    break;
                }
                case NONE: {
                    // continue with updates below
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid async action " + asyncActionInProgress);
            }
        } catch (TimeoutException | IOException ex) {
            // re-throw as unchecked exception
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException | TimeoutException " + eMessage);
        }

        // Game Controller 1
        updateToggleSpeed();
        //**TODO updateHangUp();
        //**TODO updateHangDown();

        // Game Controller 2
        //**TODO updateIntake();
        //**TODO updateOuttake();
        updateDeliveryLevel1();
        updateDeliveryLevel2();
        //**TODO 10/31/23 disable for now updateDeliveryLevel3();
        updateGoToSafe();
        updateGoToGround();
    }

    private void updateToggleSpeed() {
        if (toggleSpeed.is(FTCButton.State.TAP)) {
            FTCToggleButton.ToggleState newToggleState = toggleSpeed.toggle();
            if (newToggleState == FTCToggleButton.ToggleState.A) {
                driveTrainVelocity = driveTrainVelocityHigh;
            } else driveTrainVelocity = driveTrainVelocityLow;
        }
    }

    private boolean velocityChanged() {
        if (driveTrainVelocity == previousDriveTrainVelocity)
            return false;

        previousDriveTrainVelocity = driveTrainVelocity;
        return true;
    }

    private void updateDeliveryLevel1() {
        if (deliveryLevel1.is(FTCButton.State.TAP)) {
            move_to_delivery_level(Elevator.ElevatorLevel.LEVEL_1);
        }
    }

    private void updateDeliveryLevel2() {
        if (deliveryLevel2.is(FTCButton.State.TAP)) {
            move_to_delivery_level(Elevator.ElevatorLevel.LEVEL_2);
        }
    }

    //**TODO 10/31/23 disable for now
    /*
    private void updateDeliveryLevel3() {
        if (deliveryLevel3.is(FTCButton.State.TAP)) {
            move_to_delivery_level(Elevator.ElevatorLevel.LEVEL_3);
        }
    }
    */

    private void updateGoToSafe() {
        if (goToSafe.is(FTCButton.State.TAP)) {
            if (asyncActionInProgress != AsyncAction.NONE) {
                RobotLogCommon.v(TAG, "Error: current elevator level " + currentElevatorLevel + ", asyncActionInProgress " + asyncActionInProgress);
                return;
            }

            if (currentElevatorLevel == Elevator.ElevatorLevel.SAFE)
                return; // already there

            if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND) { // upward movement?
                if (currentBoomLevel != Boom.BoomLevel.REST) // sanity check
                    throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator from ground to safe when the boom is not at rest");

                elevatorMotion.moveDualMotors(robot.elevator.safe, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                currentElevatorLevel = Elevator.ElevatorLevel.SAFE;
            } else { // downward movement
                if (currentElevatorLevel == Elevator.ElevatorLevel.CLEAR) {
                    if (currentBoomLevel != Boom.BoomLevel.REST) // sanity check
                        throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator from clear to safe when the boom is not at rest");

                    elevatorMotion.moveDualMotors(robot.elevator.safe, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
                    currentElevatorLevel = Elevator.ElevatorLevel.SAFE;
                } else { // moving down from levels 1, 2, 3
                    // Move the elevator down to CLEAR and the boom to REST simultaneously.
                    // Then move the elevator down to SAFE.
                    async_move_elevator_down_and_boom_in(elevatorVelocity, boomVelocity);
                }
            }
        }
    }

    private void updateGoToGround() {
        if (goToGround.is(FTCButton.State.TAP)) {
            if (asyncActionInProgress != AsyncAction.NONE) {
                RobotLogCommon.v(TAG, "Error: current elevator level " + currentElevatorLevel + ", asyncActionInProgress " + asyncActionInProgress);
                return;
            }

            if (currentElevatorLevel == Elevator.ElevatorLevel.GROUND)
                return; // already there

            if (!(currentElevatorLevel == Elevator.ElevatorLevel.CLEAR || currentElevatorLevel == Elevator.ElevatorLevel.SAFE))
                throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator to ground from " + currentElevatorLevel);

            elevatorMotion.moveDualMotors(robot.elevator.ground, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            currentElevatorLevel = Elevator.ElevatorLevel.GROUND;
        }
    }

    private void move_to_delivery_level(Elevator.ElevatorLevel pDeliverToLevel) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Request to move the elevator elevator to " + pDeliverToLevel + " but an operation is already in progress");
            RobotLogCommon.v(TAG, "Current elevator level " + currentElevatorLevel + ", asyncActionInProgress " + asyncActionInProgress);
            return;
        }

        // Validate the delivery level.
        if (!(pDeliverToLevel == Elevator.ElevatorLevel.LEVEL_1 ||
                pDeliverToLevel == Elevator.ElevatorLevel.LEVEL_2 ||
                pDeliverToLevel == Elevator.ElevatorLevel.LEVEL_3)) {
            RobotLogCommon.v(TAG, "Invalid request to deliver at elevator " + pDeliverToLevel);
            return;
        }

        // Movement must start at the SAFE level and then go up to CLEAR
        // before the boom can be deployed.
        if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE) {
            RobotLogCommon.v(TAG, "Move to delivery level may not start at elevator " + currentElevatorLevel);
            return;
        }

        if (currentBoomLevel != Boom.BoomLevel.REST) // sanity check
            throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator to clear when the boom is not at rest");

        //**TODO checkRakeLifterDown();

        elevatorMotion.moveDualMotors(robot.elevator.clear, elevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
        currentElevatorLevel = Elevator.ElevatorLevel.CLEAR;

        linearOpMode.telemetry.addLine("Position for delivery at " + pDeliverToLevel);
        linearOpMode.telemetry.update();

        switch (pDeliverToLevel) {
            case LEVEL_1: {
                async_move_elevator_up_and_boom_out(Objects.requireNonNull(robot.elevator).level_1, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_1,
                        Objects.requireNonNull(robot.boom).level_1, boomVelocity, Boom.BoomLevel.LEVEL_1);
                break;
            }
            case LEVEL_2: {
                async_move_elevator_up_and_boom_out(Objects.requireNonNull(robot.elevator).level_2, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_2,
                        Objects.requireNonNull(robot.boom).level_2, boomVelocity, Boom.BoomLevel.LEVEL_2);
                break;
            }
            case LEVEL_3: {
                async_move_elevator_up_and_boom_out(robot.elevator.level_3, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_3,
                        Objects.requireNonNull(robot.boom).level_3, boomVelocity, Boom.BoomLevel.LEVEL_3);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Invalid elevator level " + pDeliverToLevel);
        }
    }

    private void async_move_elevator_up_and_boom_out(int pElevatorPosition, double pElevatorVelocity, Elevator.ElevatorLevel pElevatorLevelOnCompletion,
                                                     int pBoomPosition, double pBoomVelocity, Boom.BoomLevel pBoomLevelOnCompletion) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Async movement already in progress: " + asyncActionInProgress);
            return;
        }

        //**TODO checkRakeLifterDown();

        Callable<Elevator.ElevatorLevel> callableMoveElevatorUp = () -> {
            elevatorMotion.moveDualMotors(pElevatorPosition, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return pElevatorLevelOnCompletion;
        };

        Callable<Boom.BoomLevel> callableMoveBoom = () -> {
            boomMotion.moveSingleMotor(pBoomPosition, pBoomVelocity, SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            return pBoomLevelOnCompletion;
        };

        asyncActionInProgress = AsyncAction.MOVE_ELEVATOR_UP_AND_BOOM_OUT;
        RobotLogCommon.v(TAG, "Async move elevator up in progress");
        asyncMoveElevator = Threading.launchAsync(callableMoveElevatorUp);
        RobotLogCommon.v(TAG, "Async arm out in progress");
        asyncMoveBoom = Threading.launchAsync(callableMoveBoom);
        RobotLogCommon.v(TAG, "Async move elevator up and boom out in progress");
    }

    private void async_move_elevator_down_and_boom_in(double pElevatorVelocity, double pBoomVelocity) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Async movement already in progress: " + asyncActionInProgress);
            return;
        }

        if (!(currentElevatorLevel == Elevator.ElevatorLevel.LEVEL_1 ||
                currentElevatorLevel == Elevator.ElevatorLevel.LEVEL_2 ||
                currentElevatorLevel == Elevator.ElevatorLevel.LEVEL_3)) // sanity check
            throw new AutonomousRobotException(TAG, "Illegal attempt to move the elevator down from a level that is not 1, 2, or 3");

        //**TODO checkRakeLifterDown();

        Callable<Boom.BoomLevel> callableMoveBoom = () -> {
            boomMotion.moveSingleMotor(robot.boom.rest, pBoomVelocity, SingleMotorMotion.MotorAction.MOVE_AND_STOP);
            return Boom.BoomLevel.REST;
        };

        Callable<Elevator.ElevatorLevel> callableMoveElevatorDown = () -> {
            elevatorMotion.moveDualMotors(robot.elevator.clear, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return Elevator.ElevatorLevel.CLEAR;
        };

        asyncActionInProgress = AsyncAction.MOVE_ELEVATOR_DOWN_AND_BOOM_IN;
        asyncMoveBoom = Threading.launchAsync(callableMoveBoom);
        asyncMoveElevator = Threading.launchAsync(callableMoveElevatorDown);
        RobotLogCommon.v(TAG, "Async move elevator down and retract boom in progress");
    }

}
