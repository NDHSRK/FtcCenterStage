package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.EnumSet;

// N-way toggle based on an EnumSet<E>
public class FTCToggleButtonNWay<E extends Enum<E>> extends FTCButton {

    //private final E[] toggleValues;
    private final ArrayList<E> toggleArray = new ArrayList<>();
    private int toggleIndex = 0;

    public FTCToggleButtonNWay(LinearOpMode pLinear, ButtonValue pButtonValue, EnumSet<E> pToggleSet) {
        super(pLinear, pButtonValue);
        //illegal cast from Object[] to E[] toggleValues = (E[]) pToggleSet.toArray();
        toggleArray.addAll(pToggleSet);
    }

    public E toggle() {
        toggleIndex = (toggleIndex < (toggleArray.size() - 1)) ? ++toggleIndex : 0;
        return toggleArray.get(toggleIndex);
    }

    public E getToggleState() {
        return toggleArray.get(toggleIndex);
    }
}
