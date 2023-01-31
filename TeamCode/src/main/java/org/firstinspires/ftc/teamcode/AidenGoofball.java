package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Aiden's Autonomous Cone Test")
public class AidenGoofball extends LinearOpMode {
    private CassHardware robot = new CassHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.driveDistance(24, AidenDirections.RIGHT);
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
