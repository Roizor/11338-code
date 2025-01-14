package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.AidenDirections;
import org.firstinspires.ftc.teamcode.helpers.CassHardware;

// This is our TeleOp mode.
@TeleOp(name="Working TeleOp (Cass)")
public class CassTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    CassHardware robot = new CassHardware(this);

    @Override

    public void runOpMode() {
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        double arm = 0;
        double handOffset = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
             In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            This way it's also easy to just drive straight, or just turn. */
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            strafe  =  gamepad1.left_stick_x;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(drive, turn , strafe);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            if (gamepad2.left_bumper) robot.setHand(AidenDirections.OPEN);

            if (gamepad2.right_bumper)robot.setHand(AidenDirections.CLOSED);

            if(gamepad2.triangle)robot.setHand(AidenDirections.ALLOPEN); // This is our newest addition: setting the claw to all the way open from Triangle being pressed

            if (gamepad2.dpad_up)
            {
                // Make the arm go up
                robot.motorLift.setPower(-0.5);
            }
            else if (gamepad2.dpad_down)
            {
                // Make the arm go down
                robot.motorLift.setPower(0.1);
            }
            else
            {
                // Apply constant force to the arm so it does not get stuck
                robot.motorLift.setPower(-0.07);
            }


            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm Power",  "%.2f", arm);
            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
