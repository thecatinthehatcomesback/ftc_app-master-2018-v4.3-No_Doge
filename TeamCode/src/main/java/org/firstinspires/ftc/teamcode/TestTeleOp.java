/*
    TestTeleOp.java

    A Linear opMode class to be our teleOp testing
    method to try and solve our problems throughout
    the year.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
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
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    CatMecanumHardware robot; // use the class created for the hardware
    boolean inReverse = true;

    // constructor for class
    public TestTeleOp() {
        robot = new CatMecanumHardware();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        robot.IMUinit();
        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Go!
        runtime.reset();
        double driveSpeed;
        double leftFront = 0;
        double rightFront = 0;
        double leftBack = 0;
        double rightBack = 0;
        double SF;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

             /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Speed adjustments
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.4;
            } else {
                driveSpeed = 0.6;
            }

            //select direction
            if (gamepad1.y){
                inReverse = false;
                if(robot.isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                }
            }
            if (gamepad1.a) {
                inReverse = true;
                if(robot.isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                }
            }

            // Input for drive train
            if(inReverse) {
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

            // Calculate the scale factor to reduce the powers
            SF = robot.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each drive motor
            leftFront  = leftFront  * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack   = leftBack   * SF * driveSpeed;
            rightBack  = rightBack  * SF * driveSpeed;


            // drive //
            robot.drive(leftFront, rightFront, leftBack, rightBack);

            // Driver 1 Tail Control
            robot.tailMotor.setPower(gamepad1.left_trigger -gamepad1.right_trigger);


            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Tail control  for Driver 2
            /*
            if (gamepad2.y) {
                robot.retractTail();
            }
            if (gamepad2.x) {
                robot.extendTail();
            }
            */

            // Intake Spinning Controls
            robot.intakeServo.setPower(gamepad2.right_trigger*0.87 - gamepad2.left_trigger*0.87);

            //**  Arm controls **//
            // Lower/raise arm
            robot.armMotor.setPower(-gamepad2.right_stick_y);
            // Extend/retract arm
            robot.extenderServo.setPower(gamepad2.left_stick_x*0.8);

            // IMU Sensor
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.getPoteniometorAngle();


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            // Tail Motor Pos
            telemetry.addData("Tail Motor Position: ", robot.tailMotor.getCurrentPosition());
            // IMU Sensor
            telemetry.addData("Z Y X: ", "%.1f, %.1f, %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            // Team Marker Servo
            telemetry.addData("Marker Pos", " %.2f", robot.markerServo.getPosition());
            // Sensors
            telemetry.addData("Ultrasonic Level:", "%.3f", robot.landerSeer.getDistance(DistanceUnit.CM));
            telemetry.addData("Potentiometer Voltage?", robot.potentiometer.getVoltage());
            telemetry.addData("Pot Ang:", robot.getPoteniometorAngle());
            telemetry.addData("FrontLeft: ", "  R %3d,  G %3d,  B %3d,",
                    robot.frontLeftColor.red(), robot.frontLeftColor.green(), robot.frontLeftColor.blue());

            telemetry.addData("Pattern", "%s",robot.pattern.toString());
            telemetry.update();
        }
    }
}
