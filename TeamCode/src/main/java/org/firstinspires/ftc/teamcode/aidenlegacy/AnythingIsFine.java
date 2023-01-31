package org.firstinspires.ftc.teamcode.aidenlegacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.helpers.CassHardware;

@TeleOp(name = "! TFOD Test", group = "VuforiaTests")
@Disabled
public class AnythingIsFine extends LinearOpMode {
    private CassHardware robot = new CassHardware(this);
    private static final String TFOD_MODEL_ASSET  = "/storage/emulated/0/FIRST/tflitemodels/newest.tflite";
    private static final String[] LABELS = {
            "Blue",
            "Green",
            "Red"
    };
    private static final String VUFORIA_KEY =
            "AY2IJkf/////AAABmeCCrdXLFUr3oYVrqpag8TtlOopsqWN8hL7oGstTWV0iUskaReAN1Gfej2O5RIiNfjLdG6PoaUW9sumLRXBoYk0WjWill1us+dlT7XdtCpTyAcKh7hgyrnOx9E2uypCyYIUqAVLAsD2Ch+hS5YDVIBgzKTWFHmPHQEgAAEav1b/BH7jyDA9xigi2aP4OT1kHsjOl9mNUxyaq4V1ieWZpbFiWF1W1yQD1Z3S4TAd4+5vhf08gr2WrY0oKd/1my+XYD63/FCIdS17BP5v9jq9i1JtzKsUQJD/MFGk8XmMK0rag0smlTty9NLNTANAtc8gFm8xIG4tXxREz6oRUwFsgxF4tbmphDjb9Z+lH9LwDSFRP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        robot.init();

        if (tfod != null) {
            tfod.activate();
        }
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Image", recognition.getLabel()+" - "+recognition.getConfidence()*100+"% confident");
                        }
                    }
                    telemetry.update();
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Cammy");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
         tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
