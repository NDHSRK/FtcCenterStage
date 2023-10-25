package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCTeleOpDispatch;

@TeleOp(name = "PowerPlay", group = "Drive")
//@Disabled
public class CenterStageLaunch extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        FTCTeleOpDispatch.runTeleOp(RobotConstants.RunType.TELEOP, CenterStageTeleOp.class.getSimpleName(), RobotConstants.Alliance.NONE, this,
                (FTCTeleOpDispatch.TeleOpWithAllianceParameters tp) ->
                new CenterStageTeleOp(tp.alliance, tp.linearOpMode, tp.robot, tp.ftcAuto));
    }
}