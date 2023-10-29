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
    private final FTCButton minimumGear;
    private final FTCButton maximumGear;
    private final FTCButton increaseGear;
    private final FTCButton decreaseGear;

    //**TODO How do you retract the boom?
    //**TODO Can you go up or down an elevator level without moving the boom?
    private final FTCButton deliverPixelAtSelectedLevel; //**TODO changed from launchByGearValue
    private final FTCButton resetGearValue;

    // Drive train
    private double driveTrainVelocity;
    private double previousDriveTrainVelocity;
    private final double driveTrainVelocityHigh;
    private final double driveTrainVelocityLow;
    private final ParallelDrive parallelDrive;

    // Asynchronous
    private enum AsyncAction {MOVE_ELEVATOR_AND_BOOM, POSITION_ELEVATOR_AND_BOOM_AFTER_DELIVERY, NONE}

    private AsyncAction asyncActionInProgress = AsyncAction.NONE;

    private Elevator.ElevatorLevel currentElevatorLevel = Elevator.ElevatorLevel.REST;
    private final Elevator.ElevatorLevel[] elevatorLevels = Elevator.ElevatorLevel.values();
    private final int minElevatorLevel = 0;
    private final int maxElevatorLevel = elevatorLevels.length - 1;
    private int gearValue = minElevatorLevel;
    private final DualMotorMotion elevatorMotion;
    private final double elevatorVelocity;
    private CompletableFuture<Elevator.ElevatorLevel> asyncMoveElevator;

    private final SingleMotorMotion boomMotion;
    private final double boomVelocity;
    private CompletableFuture<Void> asyncMoveBoom;

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

        elevatorVelocity = Objects.requireNonNull(robot.elevator).velocity;
        boomVelocity = Objects.requireNonNull(robot.boom).velocity;

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
        deliverPixelAtSelectedLevel = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_A);
        resetGearValue = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_X);

        // D-Pad
        minimumGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_LEFT);
        maximumGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_RIGHT);
        increaseGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_UP);
        decreaseGear = new FTCButton(linearOpMode, FTCButton.ButtonValue.GAMEPAD_2_DPAD_DOWN);

        // Start the drive train in parallel.
        parallelDrive = new ParallelDrive(linearOpMode, robot.driveTrain, driveTrainVelocity);
        RobotLogCommon.c(TAG, "Finished constructing CenterStageTeleOp");
    }

    @Override
    public void runTeleOp() throws InterruptedException {
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs.
            // 12/28/2022 From the FTC SDK documentation: "whether the OpMode is
            // currently active. If this returns false, you should break out of
            // the loop in your runOpMode() method and return to its caller.
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                return;
            }

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
        minimumGear.update();
        maximumGear.update();
        increaseGear.update();
        decreaseGear.update();
        resetGearValue.update();
        deliverPixelAtSelectedLevel.update();
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
                case MOVE_ELEVATOR_AND_BOOM: {
                    if (asyncMoveElevator.isDone() && asyncMoveBoom.isDone()) {
                        currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                        Threading.getFutureCompletion(asyncMoveBoom);
                        asyncMoveElevator = null;
                        asyncMoveBoom = null;
                        asyncActionInProgress = AsyncAction.NONE;
                        RobotLogCommon.v(TAG, "Async MOVE_ELEVATOR_AND_BOOM_FOR_DELIVERY done");
                    } else // the elevator and/or the boom are still moving
                        return; // skip the updates below
                    break;
                }
                case POSITION_ELEVATOR_AND_BOOM_AFTER_DELIVERY: {
                    if (asyncMoveElevator.isDone() && asyncMoveBoom.isDone()) {
                        currentElevatorLevel = Threading.getFutureCompletion(asyncMoveElevator);
                        Threading.getFutureCompletion(asyncMoveBoom);

                        //**TODO any other actions?

                        asyncMoveElevator = null;
                        asyncMoveBoom = null;
                        asyncActionInProgress = AsyncAction.NONE;
                        RobotLogCommon.v(TAG, "Async POSITION_ELEVATOR_AND_BOOM_AFTER_DELIVERY done");
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

        //**TODO add all actions
        // Game Controller 1
        updateToggleSpeed();
        // updateHangUp();
        // updateHangDown();

        // Game Controller 2
        // updateIntake();
        // updateOuttake();
        updateMinimumGear();
        updateMaximumGear();
        updateIncreaseGear();
        updateDecreaseGear();
        updateResetGearValue();
        updateDeliverPixelAtSelectedLevel();
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

    private void updateMinimumGear() {
        if (minimumGear.is(FTCButton.State.TAP)) {
            gearValue = minElevatorLevel;
        }
    }

    private void updateMaximumGear() {
        if (maximumGear.is(FTCButton.State.TAP)) {
            gearValue = maxElevatorLevel;
        }
    }

    private void updateIncreaseGear() {
        if (increaseGear.is(FTCButton.State.TAP)) {
            if (gearValue == maxElevatorLevel)
                return; // at max
            ++gearValue;
        }
    }

    private void updateDecreaseGear() {
        if (decreaseGear.is(FTCButton.State.TAP)) {
            if (gearValue == minElevatorLevel)
                return; // at min
            --gearValue;
        }
    }

    private void updateResetGearValue() {
        if (resetGearValue.is(FTCButton.State.TAP)) {
            gearValue = minElevatorLevel;
        }
    }

    // Move the elevator from its safe position to the position set by the
    // gear selection.
    // Automatically extend the pixel delivery boom.
    private void updateDeliverPixelAtSelectedLevel() {
        if (deliverPixelAtSelectedLevel.is(FTCButton.State.TAP)) {
            RobotLogCommon.v(TAG, "Entered updateDeliverPixelAtSelectedLevel");

            if (currentElevatorLevel != Elevator.ElevatorLevel.SAFE ||
                    asyncActionInProgress != AsyncAction.NONE) {
                RobotLogCommon.v(TAG, "Error: current elevator level " + currentElevatorLevel + ", asyncActionInProgress " + asyncActionInProgress);
                return;
            }

            //**TODO checkRakeLifterDown();

            // Can we deliver from the elected level?
            Elevator.ElevatorLevel deliverToLevel = elevatorLevels[gearValue];
            if (!(deliverToLevel == Elevator.ElevatorLevel.LEVEL_1 ||
                    deliverToLevel == Elevator.ElevatorLevel.LEVEL_2 ||
                    deliverToLevel == Elevator.ElevatorLevel.LEVEL_3)) {
                RobotLogCommon.v(TAG, "Invalid request to deliver at elevator " + deliverToLevel);
                return;
            }

            switch (deliverToLevel) {
                case LEVEL_1: {
                    async_move_elevator_up_and_boom_out(Objects.requireNonNull(robot.elevator).level_1, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_1,
                            Objects.requireNonNull(robot.boom).level_1, boomVelocity);
                    break;
                }
                case LEVEL_2: {
                    async_move_elevator_up_and_boom_out(Objects.requireNonNull(robot.elevator).level_2, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_2,
                            Objects.requireNonNull(robot.boom).level_2, boomVelocity);
                    break;
                }
                case LEVEL_3: {
                    async_move_elevator_up_and_boom_out(robot.elevator.level_3, elevatorVelocity, Elevator.ElevatorLevel.LEVEL_3,
                            Objects.requireNonNull(robot.boom).level_3, boomVelocity);
                    break;
                }
                default:
                    throw new AutonomousRobotException(TAG, "Invalid elvevator level " + deliverToLevel);
            }
        }
    }

    private void updateRetractAndDescend() {
        if (deliverPixelAtSelectedLevel.is(FTCButton.State.TAP)) { //**TODO WRONG!!
            RobotLogCommon.v(TAG, "Entered updateRetractAndDescend");
            if (currentElevatorLevel == Elevator.ElevatorLevel.REST || currentElevatorLevel == Elevator.ElevatorLevel.SAFE) {
                RobotLogCommon.v(TAG, "Error: Attempted to updateRetractAndDescend while currentElevatorLevel is REST/SAFE");
                return;
            }

            if (asyncActionInProgress != AsyncAction.NONE) {
                RobotLogCommon.v(TAG, "Error: current elevator level " + currentElevatorLevel + ", asyncActionInProgress " + asyncActionInProgress);
                return;
            }

            //**TODO checkRakeLifterDown();

            // Pull the horizontal arm back to its rest position and lower
            // the elevator to its safe position.
            async_move_boom_in_and_elevator_down(Objects.requireNonNull(robot.boom).rest, boomVelocity,
                    Objects.requireNonNull(robot.elevator).safe, elevatorVelocity);
        }
    }


    private void async_move_elevator_up_and_boom_out(int pElevatorPosition, double pElevatorVelocity, Elevator.ElevatorLevel pElevatorLevelOnCompletion,
                                                     int pBoomPosition, double pBoomVelocity) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Async movement already in progress: " + asyncActionInProgress);
            return;
        }

        Callable<Elevator.ElevatorLevel> callableMoveElevatorUp = () -> {
            elevatorMotion.moveDualMotors(pElevatorPosition, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return pElevatorLevelOnCompletion;
        };

        Callable<Void> callableMoveBoom = () -> {
            boomMotion.moveSingleMotor(pBoomPosition, pBoomVelocity, SingleMotorMotion.MotorAction.MOVE_AND_HOLD_VELOCITY);
            return null;
        };

        asyncActionInProgress = AsyncAction.MOVE_ELEVATOR_AND_BOOM;
        RobotLogCommon.v(TAG, "Async move elevator up in progress");
        asyncMoveElevator = Threading.launchAsync(callableMoveElevatorUp);
        RobotLogCommon.v(TAG, "Async arm out in progress");
        asyncMoveBoom = Threading.launchAsync(callableMoveBoom);
        RobotLogCommon.v(TAG, "Async move elevator up and boom out in progress");
    }

    private void async_move_boom_in_and_elevator_down(int pBoomPosition, double pBoomVelocity,
                                                      int pElevatorPosition, double pElevatorVelocity) {
        if (asyncActionInProgress != AsyncAction.NONE) {
            RobotLogCommon.d(TAG, "Async movement already in progress: " + asyncActionInProgress);
            return;
        }

        Callable<Void> callableMoveBoom = () -> {
            boomMotion.moveSingleMotor(pBoomPosition, pBoomVelocity, SingleMotorMotion.MotorAction.MOVE_AND_STOP);
            return null;
        };

        Callable<Elevator.ElevatorLevel> callableMoveElevatorDown = () -> {
            elevatorMotion.moveDualMotors(pElevatorPosition, pElevatorVelocity, DualMotorMotion.DualMotorAction.MOVE_AND_HOLD_VELOCITY);
            return Elevator.ElevatorLevel.SAFE;
        };

        asyncActionInProgress = AsyncAction.POSITION_ELEVATOR_AND_BOOM_AFTER_DELIVERY;
        asyncMoveBoom = Threading.launchAsync(callableMoveBoom);
        linearOpMode.sleep(500); //**TODO ??
        asyncMoveElevator = Threading.launchAsync(callableMoveElevatorDown);
        RobotLogCommon.v(TAG, "Async retract boom and move elevator down in progress");
    }

}
