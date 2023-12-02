package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

@Autonomous(name = "BLUE_A2", group = "TeamCode")
//@Disabled
public class BLUE_A2 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch.runAuto(RobotConstants.RunType.AUTONOMOUS,
                RobotConstantsCenterStage.OpMode.BLUE_A2,
                RobotConstants.Alliance.BLUE, this);
    }
}


