package org.firstinspires.ftc.teamcode.aidenlegacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.AidenDirections;
import org.firstinspires.ftc.teamcode.helpers.CassHardware;
/**
 * This is our very first working autonomous mode that does not use vision whatsoever
 */
@Autonomous(name = "Aiden's Autonomous Zone 2 Park")
@Disabled
public class AidenAutonomousZ2 extends LinearOpMode {
    private CassHardware robot = new CassHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.driveDistance(24, AidenDirections.RIGHT);
        while (opModeIsActive()) {
            telemetry.addData("Going to", "Zone 2!");
            telemetry.addData("Bob @", robot.motorLeftFront.getCurrentPosition());
            telemetry.addData("Timothy @", robot.motorRightFront.getCurrentPosition());
            telemetry.addData("Bleff @", robot.motorBackLeft.getCurrentPosition());
            telemetry.addData("Josh @", robot.motorBackRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
