package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;

// Class for binding a binary button on an FTC Gamepad and for
// tracking its state.
//
// The original had a state for DOUBLE_TAP which was never used.
// DOUBLE_TAP would never be detected for any operation that ran
// more than the double-tap interval, which was 500ms. The HELD
// state is needed to prevent a short-lived operation, such as a
// toggle flip, from being executed more than once.
public class FTCButton {

    public static final String TAG = FTCButton.class.getSimpleName();

    // All boolean Gamepad buttons.
    public enum ButtonValue {
        GAMEPAD_1_A, GAMEPAD_1_B,
        GAMEPAD_1_DPAD_DOWN, GAMEPAD_1_DPAD_LEFT, GAMEPAD_1_DPAD_RIGHT, GAMEPAD_1_DPAD_UP,
        GAMEPAD_1_LEFT_BUMPER, GAMEPAD_1_LEFT_STICK_BUTTON, GAMEPAD_1_RIGHT_BUMPER, GAMEPAD_1_RIGHT_STICK_BUTTON,
        GAMEPAD_1_START, GAMEPAD_1_X, GAMEPAD_1_Y,
        GAMEPAD_2_A, GAMEPAD_2_B,
        GAMEPAD_2_DPAD_DOWN, GAMEPAD_2_DPAD_LEFT, GAMEPAD_2_DPAD_RIGHT, GAMEPAD_2_DPAD_UP,
        GAMEPAD_2_LEFT_BUMPER, GAMEPAD_2_LEFT_STICK_BUTTON, GAMEPAD_2_RIGHT_BUMPER, GAMEPAD_2_RIGHT_STICK_BUTTON,
        GAMEPAD_2_START, GAMEPAD_2_X, GAMEPAD_2_Y,
    }

    public enum State { // button states
        TAP,    // button pressed
        HELD,   // button pressed for more than one cycle
        UP,     // button released
        OFF,    // button not pressed for more than once cycle
    }

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final ButtonValue buttonValue;
    private State state;

    public FTCButton(LinearOpMode pLinear, ButtonValue pButtonValue) {
        gamepad1 = pLinear.gamepad1;
        gamepad2 = pLinear.gamepad2;
        buttonValue = pButtonValue; // Associate this instance with this button
        state = State.OFF;
    }

    public State getState() {
        return state;
    }

    public State update() {
        boolean buttonPressed = false;

        switch (buttonValue) {
            case GAMEPAD_1_A: {
                buttonPressed = gamepad1.a;
                break;
            }
            case GAMEPAD_1_B: {
                buttonPressed = gamepad1.b;
                break;
            }
            case GAMEPAD_1_DPAD_DOWN: {
                buttonPressed = gamepad1.dpad_down;
                break;
            }
            case GAMEPAD_1_DPAD_LEFT: {
                buttonPressed = gamepad1.dpad_left;
                break;
            }
            case GAMEPAD_1_DPAD_RIGHT: {
                buttonPressed = gamepad1.dpad_right;
                break;
            }
            case GAMEPAD_1_DPAD_UP: {
                buttonPressed = gamepad1.dpad_up;
                break;
            }
            case GAMEPAD_1_LEFT_BUMPER: {
                buttonPressed = gamepad1.left_bumper;
                break;
            }
            case GAMEPAD_1_LEFT_STICK_BUTTON: {
                buttonPressed = gamepad1.left_stick_button;
                break;
            }
            case GAMEPAD_1_RIGHT_BUMPER: {
                buttonPressed = gamepad1.right_bumper;
                break;
            }
            case GAMEPAD_1_RIGHT_STICK_BUTTON: {
                buttonPressed = gamepad1.right_stick_button;
                break;
            }
            case GAMEPAD_1_START: {
                buttonPressed = gamepad1.start;
                break;
            }
            case GAMEPAD_1_X: {
                buttonPressed = gamepad1.x;
                break;
            }
            case GAMEPAD_1_Y: {
                buttonPressed = gamepad1.y;
                break;
            }
            //
            case GAMEPAD_2_A: {
                buttonPressed = gamepad2.a;
                break;
            }
            case GAMEPAD_2_B: {
                buttonPressed = gamepad2.b;
                break;
            }
            case GAMEPAD_2_DPAD_DOWN: {
                buttonPressed = gamepad2.dpad_down;
                break;
            }
            case GAMEPAD_2_DPAD_LEFT: {
                buttonPressed = gamepad2.dpad_left;
                break;
            }
            case GAMEPAD_2_DPAD_RIGHT: {
                buttonPressed = gamepad2.dpad_right;
                break;
            }
            case GAMEPAD_2_DPAD_UP: {
                buttonPressed = gamepad2.dpad_up;
                break;
            }
            case GAMEPAD_2_LEFT_BUMPER: {
                buttonPressed = gamepad2.left_bumper;
                break;
            }
            case GAMEPAD_2_LEFT_STICK_BUTTON: {
                buttonPressed = gamepad2.left_stick_button;
                break;
            }
            case GAMEPAD_2_RIGHT_BUMPER: {
                buttonPressed = gamepad2.right_bumper;
                break;
            }
            case GAMEPAD_2_RIGHT_STICK_BUTTON: {
                buttonPressed = gamepad2.right_stick_button;
                break;
            }
            case GAMEPAD_2_START: {
                buttonPressed = gamepad2.start;
                break;
            }
            case GAMEPAD_2_X: {
                buttonPressed = gamepad2.x;
                break;
            }
            case GAMEPAD_2_Y: {
                buttonPressed = gamepad2.y;
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "No case for button " + buttonValue);
        }

        if (buttonPressed) {
            if (state == State.OFF || state == State.UP) {
                state = State.TAP;
            } else {
                state = State.HELD;
            }
        }
        else {
            if (state == State.HELD || state == State.TAP) {
                state = State.UP;
            }
            else {
                state = State.OFF;
            }
        }

        return state;
    }

    public boolean is(State pState) {
        return state == pState;
    }

}
