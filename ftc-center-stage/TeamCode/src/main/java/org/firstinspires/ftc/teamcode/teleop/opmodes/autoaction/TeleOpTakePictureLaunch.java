package org.firstinspires.ftc.teamcode.teleop.opmodes.autoaction;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCTeleOpDispatch;

@TeleOp(name = "TeleOpTakePicture", group = "AutoAction")
@Disabled
public class TeleOpTakePictureLaunch extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        FTCTeleOpDispatch.runTeleOp(RobotConstants.RunType.TELEOP_NO_DRIVE_WITH_EMBEDDED_AUTONOMOUS, TeleOpTakePicture.class.getSimpleName(), RobotConstants.Alliance.NONE, this,
                (FTCTeleOpDispatch.TeleOpWithAllianceParameters tp) ->
                        new TeleOpTakePicture(tp.alliance, tp.linearOpMode, tp.robot, tp.ftcAuto));
    }
}