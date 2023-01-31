package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Aiden's Autonomous")
public class AidenAutonomous extends LinearOpMode
{
    OpenCvCamera camera;
    CassHardware robot = new CassHardware(this);
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ZONE1_ID = 19; // Tag ID 18 from the 36h11 family
    int ZONE2_ID = 18;
    int ZONE3_ID = 17;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cammy"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        robot.init();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ZONE1_ID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    } else if(tag.id == ZONE2_ID) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    } else if(tag.id == ZONE3_ID) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            // Push cone into terminal
            robot.driveDistance(24, AidenDirections.RIGHT);
            robot.driveDistance(24, AidenDirections.LEFT);
            coneToSmallJunction();
            telemetry.addLine("Zone 2");
            // Now lets park in zone 2
            robot.driveDistance(24, AidenDirections.LEFT);
            robot.driveDistance(24, AidenDirections.FORWARDS);
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // Push cone into terminal
            robot.driveDistance(28, AidenDirections.RIGHT);
            // Go up a square
            robot.driveDistance(24, AidenDirections.FORWARDS);
            // Back into zone 2
            robot.driveDistance(26, AidenDirections.LEFT);
            // Fit back to start, now lets run cone to small junction.
            robot.driveDistance(24, AidenDirections.BACKWARDS);
            coneToSmallJunction();
            // Reset position, now lets go park in a zone.
            if(tagOfInterest.id == ZONE1_ID) {
                telemetry.addLine("Zone 1");
                robot.driveDistance(24, AidenDirections.RIGHT);
                robot.driveDistance(24, AidenDirections.FORWARDS);
            } else if(tagOfInterest.id == ZONE2_ID) {
                telemetry.addLine("Zone 2");
                robot.driveDistance(24, AidenDirections.FORWARDS);
            } else if(tagOfInterest.id == ZONE3_ID) {
                telemetry.addLine("Zone 3");
                robot.driveDistance(24, AidenDirections.LEFT);
                robot.driveDistance(28, AidenDirections.FORWARDS);
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    void coneToSmallJunction() {
        robot.setHand(AidenDirections.OPEN);
        robot.driveDistance(46, AidenDirections.FORWARDS);
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
        sleep(650);
        robot.setHand(AidenDirections.OPEN);
        robot.Evel.setPower(1);
        sleep(150);
        robot.driveDistance(4, AidenDirections.FORWARDS);
        robot.Evel.setPower(0);
        robot.driveRobot(-0.5, -1, 0.25);
        sleep(950);
        robot.driveDistance(12, AidenDirections.BACKWARDS);
        robot.driveDistance(24, AidenDirections.BACKWARDS);
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addLine("Zone 1:"+ZONE1_ID + "\nZone 2:"+ZONE2_ID+"\nZone 3:"+ZONE3_ID);
    }
}