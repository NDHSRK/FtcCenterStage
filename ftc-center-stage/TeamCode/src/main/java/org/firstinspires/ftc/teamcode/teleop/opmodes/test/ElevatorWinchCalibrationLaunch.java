package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCTeleOpDispatch;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.CenterStageTeleOp;

// Calibration of the closely correlated elevator and winch
// is built into the main TeleOp OpMode, CenterStageTeleOp.
@TeleOp(name = "ElevatorWinchCalibration", group = "Test")
@Disabled
public class ElevatorWinchCalibrationLaunch extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        FTCTeleOpDispatch.runTeleOp(RobotConstants.RunType.TELEOP, CenterStageTeleOp.class.getSimpleName(), RobotConstants.Alliance.NONE, this,
                (FTCTeleOpDispatch.TeleOpWithAllianceParameters tp) ->
                new CenterStageTeleOp(tp.alliance, tp.linearOpMode, tp.robot, tp.ftcAuto));
    }
}