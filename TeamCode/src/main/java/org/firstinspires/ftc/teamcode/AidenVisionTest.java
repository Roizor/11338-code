package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "AidenAutoCamra", group = "VuforiaTests")
public class AidenVisionTest extends LinearOpMode {
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private CassHardware robot = new CassHardware(this);
    private String[] labels = {"0 Red", "1 Green", "2 Blue"};

    @Override
    public void runOpMode() {
        List<Recognition> recognitions;

        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        robot.init();

        vuforiaPOWERPLAY.initialize(
                "AY2IJkf/////AAABmeCCrdXLFUr3oYVrqpag8TtlOopsqWN8hL7oGstTWV0iUskaReAN1Gfej2O5RIiNfjLdG6PoaUW9sumLRXBoYk0WjWill1us+dlT7XdtCpTyAcKh7hgyrnOx9E2uypCyYIUqAVLAsD2Ch+hS5YDVIBgzKTWFHmPHQEgAAEav1b/BH7jyDA9xigi2aP4OT1kHsjOl9mNUxyaq4V1ieWZpbFiWF1W1yQD1Z3S4TAd4+5vhf08gr2WrY0oKd/1my+XYD63/FCIdS17BP5v9jq9i1JtzKsUQJD/MFGk8XmMK0rag0smlTty9NLNTANAtc8gFm8xIG4tXxREz6oRUwFsgxF4tbmphDjb9Z+lH9LwDSFRP", // vuforiaLicenseKey
                robot.Cammy, // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                false
        );
//        tfod.useModelFromFile("/storage/emulated/0/model_unquant.tflite", labels, false, false, 1);
        tfod.useDefaultModel();
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true, false);
        tfod.activate();
        vuforiaPOWERPLAY.setActiveCamera(robot.Cammy);
        vuforiaPOWERPLAY.activate();
        tfod.setZoom(1, 16 / 9);
        waitForStart();
        while (opModeIsActive()) {
            // Only run the code once because we don't need it to continue to move once its parked.
            recognitions = tfod.getRecognitions();
            if (JavaUtil.listLength(recognitions) == 0) {
                telemetry.addData("Object Detection", "No items detected.");
            } else {
                moveToRecognition(recognitions.get(0).getLabel());
            }
            telemetry.update();
        }
        tfod.deactivate();
        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    // nice
    private void moveToRecognition(String recoglabel) {
        if (recoglabel.equals("0 Red")) {
            robot.driveDistance(24, AidenDirections.LEFT);
            robot.driveDistance(24, AidenDirections.FORWARDS);
            sleep(1500);
            robot.driveDistance(24, AidenDirections.BACKWARDS);
            robot.driveDistance(24, AidenDirections.RIGHT);
        } else if (recoglabel.equals("1 Green")) {
            robot.driveDistance(24, AidenDirections.FORWARDS);
            sleep(1500);
            robot.driveDistance(24, AidenDirections.BACKWARDS);
        } else if (recoglabel.equals("2 Blue")) {
            robot.driveDistance(24, AidenDirections.RIGHT);
            robot.driveDistance(24, AidenDirections.FORWARDS);
            sleep(1500);
            robot.driveDistance(24, AidenDirections.BACKWARDS);
            robot.driveDistance(24, AidenDirections.LEFT);
        }
    }
}
