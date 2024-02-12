package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Simple A/B toggle that starts in position A.
public class FTCToggleButton extends FTCButton {

    public enum ToggleState {A, B}

    private ToggleState toggleState = ToggleState.A;

    public FTCToggleButton(LinearOpMode pLinear, ButtonValue pButtonValue) {
        super(pLinear, pButtonValue);
    }

    public ToggleState toggle() {
        toggleState = (toggleState == ToggleState.A) ? ToggleState.B : ToggleState.A;
        return toggleState;
    }

    public ToggleState getToggleState() {
        return toggleState;
    }
}
