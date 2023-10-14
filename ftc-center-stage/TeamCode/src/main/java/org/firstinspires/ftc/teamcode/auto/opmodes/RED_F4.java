package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

@Autonomous(name = "RED_F4", group = "TeamCode")
//@Disabled
public class RED_F4 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch.runAuto(RobotConstants.RunType.AUTONOMOUS,
                RobotConstantsCenterStage.OpMode.RED_F4,
                RobotConstants.Alliance.RED, this);
    }
}


