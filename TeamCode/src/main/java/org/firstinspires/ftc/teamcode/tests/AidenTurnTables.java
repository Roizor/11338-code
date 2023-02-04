package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.AidenDirections;
import org.firstinspires.ftc.teamcode.helpers.CassHardware;

@Autonomous(name = "Aiden's Small Junction Test")
public class AidenTurnTables extends LinearOpMode {
    private CassHardware robot = new CassHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.setHand(AidenDirections.OPEN);
        robot.driveDistance(41, AidenDirections.FORWARDS);
        robot.driveRobot(0.5, 1, 0.25);
        sleep(850);
        robot.driveDistance(1, AidenDirections.FORWARDS);
        robot.driveDistance(12, AidenDirections.FORWARDS);
        robot.Evel.setPower(-1);
        sleep(250);
        robot.Evel.setPower(0);
        robot.setHand(AidenDirections.CLOSED);
        sleep(350);
        robot.Evel.setPower(-1);
        sleep(250);
        robot.Evel.setPower(1);
        sleep(250);
        robot.Evel.setPower(0);
        robot.driveDistance(17, AidenDirections.BACKWARDS);
        robot.driveDistance(12, AidenDirections.RIGHT);
        robot.driveDistance(2, AidenDirections.FORWARDS);
        robot.Evel.setPower(-1);
        sleep(550);
        robot.setHand(AidenDirections.OPEN);
        robot.Evel.setPower(1);
        sleep(150);
        robot.driveDistance(4, AidenDirections.FORWARDS);
        robot.Evel.setPower(0);
        robot.driveRobot(-0.5, -1, 0.25);
        sleep(850);
        robot.driveDistance(12, AidenDirections.BACKWARDS);
        robot.driveDistance(24, AidenDirections.BACKWARDS);
        this.stop();
        while (opModeIsActive()) {
            telemetry.addData("Going to", "Zone 2!");
            telemetry.addData("Bob @", robot.Bob.getCurrentPosition());
            telemetry.addData("Timothy @", robot.Timothy.getCurrentPosition());
            telemetry.addData("Bleff @", robot.Bleff.getCurrentPosition());
            telemetry.addData("Josh @", robot.Josh.getCurrentPosition());
            telemetry.update();
        }
    }
}
