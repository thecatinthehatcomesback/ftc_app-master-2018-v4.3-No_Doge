/**
 DemoVisionOpMode.java

 A Linear OpMode class to be


 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;


@Autonomous(name="Intermediate Autonomous", group="CatAuto")
public class DemoVisionOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    public HardwareMap  hwMap       = null;

    // Motors
    public DcMotor  leftFrontMotor  = null;
    public DcMotor  rightFrontMotor = null;
    public DcMotor  leftRearMotor   = null;
    public DcMotor  rightRearMotor  = null;

    // Objects and Detectors
    private VuforiaLocalizer vuforia;
    public  TFObjectDetector tfod;

    // Variables
    int goldMineralX = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry message to signify robot is getting ready
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();

        // Init the Drive Train
        initDrive();
        // Init Machine Vision
        initVision();

        // Wait for Play
        idle();
        telemetry.addData("Status: ", "Initialized!");
        telemetry.update();
        waitForStart();

        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */
        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getConfidence() > 0.5) {
                        int goldMineralX = (int) recognition.getLeft();
                        // Look for the Gold Pos and decide which side of the sampling field the gold lies
                        if (goldMineralX > 450) {
                            //***Inverted this since the camera was recently placed upside down***//
                            // Drive one way
                            telemetry.addData("Drive", "One way");
                        }
                        // Drive the other way
                        telemetry.addData("Drive", "Other way");
                    }
                }
            }
            telemetry.addData("goldMineralX", goldMineralX);
            telemetry.update();
        }

        /**
         * Plans...
         *
         * When it sees the Gold, it will chase it by:
         * Setting powers to each side in order to line up with the Gold.
         *
         * If it doesn't see the gold it will scan for it by:
         * Turning in a set direction somewhat slowly.
         */
    }
    /* Initialize standard Hardware interfaces */
    public void initDrive()  throws InterruptedException  {

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");


        // Define motor directions //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
    public void initVision() {

        /**
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // Team Specific License Key for Vuforia
        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        // Give the camera name used in Robot Config
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        // Now init the tfod
        int tfodMonitorViewId   = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        // And now ACTIVATE!!!
        tfod.activate();
    }
}
