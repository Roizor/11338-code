package org.firstinspires.ftc.teamcode.aidenlegacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.firstinspires.ftc.teamcode.helpers.AidenDirections;
import org.firstinspires.ftc.teamcode.helpers.CassHardware;

/**
 * This is a very early prototype for our autonomous vision using Vuforia instead of AprilTags.
 */
@Autonomous(name = "AutoCamra3", group = "VuforiaTests")
@Disabled
public class AutoCamra2 extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotor bleff;
    private DcMotor bright;
    private DcMotor lefron;
    private DcMotor rifron;

    Recognition recognition;
    List<Recognition> recognitions;
    int index;
    CassHardware robot = new CassHardware(this);

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        robot.init();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Cammy"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        String[] labels = {"0 Red", "1 Green", "2 Blue"};
        tfod.useModelFromFile("/storage/emulated/0/model_unquant.tflite", labels);
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(1, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfod.getRecognitions();
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                }
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        if (recognition.getLabel().equals("1 Bolt")) {
            telemetry.addData("Going To ", "Lightning Zone");
        } else if (recognition.getLabel().equals("2 Bulb")) {
            telemetry.addData("Going To ", "Bright Station");
            robot.driveDistance(24, AidenDirections.FORWARDS);
        } else {
            telemetry.addData("Going To ", "Sun Theives");
        }
    }
}
