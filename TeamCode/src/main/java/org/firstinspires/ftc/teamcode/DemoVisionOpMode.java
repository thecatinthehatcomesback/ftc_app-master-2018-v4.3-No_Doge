/**
 DemoVisionOpMode.java

 A Linear OpMode class to be


 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;


@TeleOp(name="DemoVision Autonomous", group="CatTeleOp")
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
    int goldMineralXPos     = 0;
    int goldMineralWidth    = 0;

    double forwardPower = 0.20;
    double leftPower    = 0.00;
    double rightPower   = 0.00;

    boolean needToDriveAhead    = true;
    boolean needToTurn          = false;
    boolean needToTurnLeft      = false;
    boolean needToTurnRight     = false;




    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry message to signify robot is getting ready
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();

        hwMap = hardwareMap;

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
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    // Lots of different tfod values placed into Telemetry
                    /*telemetry.addData("getLabel", recognition.getLabel());
                    telemetry.addData("getConfidence", recognition.getConfidence());
                    telemetry.addData("getLeft", recognition.getLeft());
                    telemetry.addData("getRight", recognition.getRight());
                    telemetry.addData("getWidth", recognition.getWidth());
                    telemetry.addData("estimateAngleToObject", recognition.estimateAngleToObject(AngleUnit.DEGREES));*/

                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getConfidence() > 0.5) {
                        // Define the goldMineral-variables.
                        goldMineralXPos = (int) recognition.getLeft();
                        goldMineralWidth = (int) recognition.getWidth();

                        /**
                         * SET FLAGS!
                         */
                        // Look for Gold and decide which side of the sampling field the Gold is.
                        if (goldMineralXPos > 200) {
                            needToTurnLeft = true;
                        } else {
                            needToTurnLeft = false;
                        }
                        if (goldMineralXPos < 400) {
                            needToTurnRight = true;
                        } else {
                            needToTurnRight = false;
                        }
                        // If between these values (deadzone in the middle of the robot's view),
                        // the robot doesn't need to turn and can drive ahead.
                        if (needToTurnLeft && needToTurnRight) {
                            needToTurn = false;
                        } else {
                            // Otherwise, the  robot must start turning to line up with Gold.
                            needToTurn = true;
                        }
                        // Decide whether the robot needs to drive ahead.
                        if (goldMineralWidth < 200) {
                            needToDriveAhead = true;
                        } else {
                            needToDriveAhead = false;
                        }
                        timer.reset();
                    }
                }
            }

        telemetry.addData("goldMineralXPos", goldMineralXPos);
        telemetry.addData("goldMineralWidth", goldMineralWidth);
        telemetry.addData("needToDriveAhead", needToDriveAhead);
        telemetry.addData("needToTurn", needToTurn);
        telemetry.update();
        telemetry.addData("forwardPower", forwardPower);
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
        /**
         * Plans:
         *
         * When it sees the Gold, it will chase it by:
         * Giving a constant +power to each side in order to get closer to the Gold.
         *
         * If it doesn't see the gold it will scan for it by:
         * Turning in a set direction somewhat slowly.
         *
         * Greater than 200 getWidth = stop
         *
         * if needToDriveAhead then drive ahead with one set of left/right difference powers
         * otherwise use another set of left/right difference powers to turn the bot.
         */

        // If the robot can't see the Gold, just stop and rest until it sees it again.
        if (timer.seconds() > 0.25) {
            needToDriveAhead = false;
            needToTurn = false;
        }

        if (needToDriveAhead && !needToTurn) {
            // Just drive  straight
            forwardPower = 0.30;
            leftPower = forwardPower;
            rightPower = forwardPower;
        } else if (needToDriveAhead && needToTurn) {
            // Add/Subtract some power
            forwardPower = 0.30;
            if (needToTurnRight) {
                leftPower = forwardPower + 0.15;
                rightPower = forwardPower - 0.15;
            } else {
                leftPower = forwardPower - 0.15;
                rightPower = forwardPower + 0.15;
            }
        } else if (!needToDriveAhead && needToTurn) {
            // Add/Subtract MORE power
            if (needToTurnRight) {
                leftPower =   0.25;
                rightPower = -0.25;
            } else {
                leftPower =  -0.25;
                rightPower =  0.25;
            }
        } else {
            // Stop.
            forwardPower = 0.00;
            leftPower = forwardPower;
            rightPower = forwardPower;
        }

        // DRIVE //
        leftFrontMotor.setPower(leftPower);
        leftRearMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightRearMotor.setPower(rightPower);
        }
    }
    /* Initialize standard Hardware interfaces */
    public void initDrive()  throws InterruptedException  {

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");


        // Define motor directions //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);


        // Define motor modes //
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0.00);
        leftRearMotor.setPower(0.00);
        rightRearMotor.setPower(0.00);
        rightRearMotor.setPower(0.00);
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
