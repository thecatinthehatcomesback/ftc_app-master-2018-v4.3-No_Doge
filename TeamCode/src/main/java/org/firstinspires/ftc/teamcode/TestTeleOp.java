/*
        TestTeleOp.java

    A Linear opMode class to be our TeleOp testing method to try
    and solve our problems throughout the year without having to
    modify the main TeleOp.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Tester TeleOp", group="CatTeleOp")

public class TestTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatMecanumHW robot; // use the class created for the hardware
    CatVisionHW eyes = new CatVisionHW();   // Use the mecanum hardware
    boolean inReverse = true;

    // constructor for class
    public TestTeleOp() {
        robot = new CatMecanumHW();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        robot.IMUinit();
        // Init our Machine Vision
        eyes.initVision(hardwareMap);

        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(robot.isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }
        // Go!
        runTime.reset();
        elapsedGameTime.reset();
        boolean tailRetracted = false;
        boolean needsToBeRetracted = true;
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        int newEncTicks = 0;
        int oldEncTicks = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Drive speed adjustments
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.4;
            } else {
                driveSpeed = 0.6;
            }

            // Direction Selection
            if (gamepad1.dpad_up){
                inReverse = false;
                if(robot.isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                }
            }
            if (gamepad1.dpad_down) {
                inReverse = true;
                if(robot.isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                }
            }

            // Input for drive train
            if (inReverse) {
                leftFront  = gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
                rightFront = gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
                leftBack   = gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
                rightBack  = gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            } else {
                leftFront  = -gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
                rightFront = -gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
                leftBack   = -gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
                rightBack  = -gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            }


            // Calculate the scale factor
            SF = robot.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each drive motor
            leftFront  = leftFront  * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack   = leftBack   * SF * driveSpeed;
            rightBack  = rightBack  * SF * driveSpeed;


            // DRIVE!!!
            robot.drive(leftFront, rightFront, leftBack, rightBack);

            // Tail Control
            robot.tailMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);





            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Intake Spinning Controls
            robot.intakeServo.setPower(gamepad2.right_trigger*0.87 - gamepad2.left_trigger*0.87);

            //**  Arm controls **//
            // Lower/Raise arm
            robot.armMotor.setPower(gamepad2.right_stick_y);
            // Extend/Retract arm
            robot.extenderMotor.setPower(gamepad2.left_stick_x * 0.8);
            // Open/Close gate
            if(gamepad2.left_bumper) {
                robot.gateClose();
            } else if (gamepad2.right_bumper) {
                robot.gateOpen();
            }


            // Driver help
            if (elapsedGameTime.seconds() > 100) {
                robot.extendTail();
            }// IMU Sensor
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            /// TODO: 2/4/2019 TEST THIS!!!
            if (tailRetracted && needsToBeRetracted) {
                // Get the current encoder value
                newEncTicks = robot.tailMotor.getCurrentPosition();
                // Set tail to retract slowly at quarter power
                robot.tailMotor.setPower(-0.4);

                // Every so often, check the values
                if (runTime.milliseconds() > 750) {
                    // Once the encoder ticks slows down or stops
                    if ((newEncTicks - oldEncTicks) > -80) {
                        // Cut power to tail
                        robot.tailMotor.setPower(0.0);
                        // Tell EVERYONE!!!
                        telemetry.addData("Status: ", "Success!!!");
                        telemetry.update();
                        robot.robotWait(3.0);
                        // End this
                        tailRetracted = true;
                        needsToBeRetracted = false;
                    }

                    // Otherwise continue as normal and reset timer
                    telemetry.addData("Status: ", "Still going...");
                    runTime.reset();
                    oldEncTicks = newEncTicks;
                }
                telemetry.addData("Current Enc Ticks: ", newEncTicks);
                telemetry.addData("Old Enc Ticks: ", oldEncTicks);
                telemetry.update();
            }

            /// TODO: 2/4/2019 After so much time pull the arm back in


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            // Tail Motor Pos
            telemetry.addData("Tail Motor Position: ", robot.tailMotor.getCurrentPosition());
            // IMU Sensor
            telemetry.addData("Z Y X: ", "%.1f, %.1f, %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            // Sensors
            telemetry.addData("Ultrasonic Level:", "%.3f", robot.landerSeer.getDistance(DistanceUnit.CM));
            telemetry.addData("FrontLeft: ", "  A: %3d,  R %3d,  G %3d,  B %3d,",
                    robot.frontLeftColor.alpha(), robot.frontLeftColor.red(),
                    robot.frontLeftColor.green(), robot.frontLeftColor.blue());
            telemetry.addData("FrontRight: ", "  A: %3d,  R %3d,  G %3d,  B %3d,",
                    robot.frontRightColor.alpha(), robot.frontRightColor.red(),
                    robot.frontRightColor.green(), robot.frontRightColor.blue());
            telemetry.addData("BackLeft: ", "  A: %3d,  R %3d,  G %3d,  B %3d,",
                    robot.rearLeftColor.alpha(), robot.rearLeftColor.red(),
                    robot.rearLeftColor.green(), robot.rearLeftColor.blue());
            telemetry.addData("BackRight: ", "  A %3d,  R %3d,  G %3d,  B %3d,",
                    robot.rearRightColor.alpha(), robot.rearRightColor.red(),
                    robot.rearRightColor.green(), robot.rearRightColor.blue());

            telemetry.addData("Pattern", "%s",robot.pattern.toString());

            telemetry.addData("Arm Encoder","%d",robot.armMotor.getCurrentPosition());
            telemetry.addData("Extend Encoder","%d",robot.extenderMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
