package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint; // get the Lint (whatever that is) warnings to shut up lol, there's an entire library for that
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class TensorFlow {
    /*
    Important: run the funny little initTfod() method to start up everything before doing any tensorflow things, thanks
     */

    private static final boolean USE_WEBCAM = true;  // true for webcam (on bot duh), false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "TeamProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite"; // make sure to add the custom model and update this
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Team Prop Red",
            "Team Prop Blue",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;



    private VisionPortal visionPortal; // The variable to store our instance of the vision portal.

// commenting this out probably broke everything but it looks useless so here we go:

//    public void detectTeamProp(Telemetry telemetry, WebcamName webcamName) {
//
//        initTfod(webcamName);
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//
//                telemetryTfod();
//
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//
//                // Save CPU resources; can resume streaming when needed.
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                // Share the CPU.
//                sleep(20);
//            }
//        }
//
//        // Save more CPU resources when camera is no longer needed.
//        visionPortal.close();
//
//    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod(WebcamName webcamName) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(webcamName);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480)); // tensorflow camera calibration for the Logitech C310 requires this resolution (which is ideal anyways)

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true); // on for testing, probably not permanent

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, false); // i've set it to be disabled because the propAlliance and propPos both start by turning on the processor, let me know if that is a bad idea

    }   // end method initTfod()

    /**
     * the code only supports one object detection, but should multiple objects be recognized it will
     * hopefully use the object with the highest confidence (our team prop)
     */


    // propAlliance checks what color the prop is
    private String propAlliance(Telemetry telemetry, Recognition recognition) {
        visionPortal.setProcessorEnabled(tfod, true); // enable the tensorflow processor to recognize team prop alliance

        // print teamprop label and confidence
        telemetry.addData("Detected", recognition.getLabel());
        telemetry.addData("Confidence:", recognition.getConfidence() * 100);
        telemetry.update();

        String alliance = null; // set the alliance string to null in preparation of team prop detection

        // set alliance variable
        if ("Team Prop Red".equals(recognition.getLabel())) {
            alliance = "red";
        } else if ("Team Prop Blue".equals(recognition.getLabel())) {
            alliance = "blue";
        } else {
            telemetry.addData("Error: Couldn't detect any objects, or incorrect label.", null);
            telemetry.update();
        }

        visionPortal.setProcessorEnabled(tfod, false); // disable the tensorflow processor to clear up resources
        return alliance;
    }

    // propPos determines the location of the prop and returns either Left, Center, or Right
    private String propPos(Telemetry telemetry, Recognition recognition) {
        visionPortal.setProcessorEnabled(tfod, true); // turn on the processor for tensorflow if not already on
        String propPos;
        double x = (recognition.getLeft() + recognition.getRight()) / 2; // calculate center x value of recognized object

        // quick math: 640/3 = 213.3333333333333, round up to 214
        // left: 0-214, middle: 214-428, right: 428-642

        telemetry.addData("Detected", recognition.getLabel());
        telemetry.addData("Confidence:", recognition.getConfidence() * 100);
        telemetry.update();

        if (x >= 0 && x <= 214) {
            propPos = "left";
            telemetry.addData("Detected team prop at left spike, color: ", recognition.getLabel());
            telemetry.update();

        } else if (x >= 215 && x <= 428) {
            propPos = "middle";
            telemetry.addData("Detected team prop on middle spike, color: ", recognition.getLabel());
            telemetry.update();

        } else if (x >= 429 && x <= 640) {
            propPos = "right";
            telemetry.addData("Detected team prop on right spike, color: ", recognition.getLabel());
            telemetry.update();
        } else {
            propPos = "fail";
            telemetry.addData("Couldn't detect any team props.", null);
            telemetry.update();
        }

        visionPortal.setProcessorEnabled(tfod, false); // disable tfod to save processing power
        return propPos;
    }

}   // end class (really FTC?)
