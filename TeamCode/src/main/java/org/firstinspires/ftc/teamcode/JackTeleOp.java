/*
        JackTeleOp.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Jack TeleOp", group="CatTeleOp")

public class JackTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatMecanumHW robot;  // Use the mecanum class created for the hardware
    boolean inReverse = true;
    boolean autoArm = false;
    boolean slowArm = false;
    // Our constructor for this class
    public JackTeleOp() {
        robot = new CatMecanumHW();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
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
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;

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

            // If the driver presses A it enters auto-mode where it
            // automatically rotates to the correct position and
            // extends at the optimal time.
            if (gamepad2.a){
                robot.armMotor.setPower(-1);
                autoArm = true;
            }

            // tests if it is in auto-mode
            if (autoArm){
                //cancels the automatic if the driver tries to move the arm themselves
                if (Math.abs(gamepad2.right_stick_y)>.55) {
                    autoArm = false;
                }
                //starts extending at the right time
                if (robot.armMotor.getCurrentPosition()<CatMecanumHW.ARM_EXTEND){
                    robot.extenderMotor.setPower(-1);
                }
                //slows down the arms movement when close to the target to not overshoot
                if (robot.armMotor.getCurrentPosition()<CatMecanumHW.ARM_SLOW&&!slowArm)
                {
                    robot.armMotor.setPower(-.7);
                    slowArm = true;
                }
                //stops arm movement if done and resets the vars
                if (robot.armMotor.getCurrentPosition()<CatMecanumHW.ARM_TELEOP){
                    robot.armMotor.setPower(0);
                    autoArm = false;
                    slowArm = false;
                }

                //if the driver isn't in auto mode it uses these methods
            }else {
                // Lower/Raise arm
                robot.armMotor.setPower(gamepad2.right_stick_y);

                // Extend/Retract arm
                robot.extenderMotor.setPower(gamepad2.left_stick_x * 0.8);
            }




            // Open/Close gate
            if(gamepad2.left_bumper) {
                robot.gateClose();
            } else if (gamepad2.right_bumper) {
                robot.gateOpen();
            }


            // Driver help
            if (elapsedGameTime.seconds() > 100) {
                robot.extendTail();
            }

            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            telemetry.addData("Tail Encoder Position:", robot.tailMotor.getCurrentPosition());
            telemetry.addData("Intake Speed:", robot.intakeServo.getPower());
            telemetry.update();
        }
    }
}
