package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

@Autonomous(name = "TestRED", group = "TeamCode")
//@Disabled
public class TestRED extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch.runAuto(RobotConstants.RunType.AUTONOMOUS,
                RobotConstantsCenterStage.OpMode.TEST,
                RobotConstants.Alliance.RED, this);
    }
}


