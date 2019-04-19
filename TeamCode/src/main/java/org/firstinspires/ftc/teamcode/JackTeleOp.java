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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Jack TeleOp", group="CatTeleOp")

public class JackTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
    boolean inReverse           = true;
    boolean autoArm             = false;
    boolean slowArm             = false;
    boolean liftingTail         = false;
    boolean overrodeLiftTail = false;

    // Our constructor for this class
    public JackTeleOp() {
        robot = new CatAsyncHW();
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
            robot.underLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            robot.underLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }


        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;

        // Run infinitely until the end of the match (driver presses STOP)
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

            // Robot Drive-Direction Selection
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
            SF = robot.drive.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each drive motor
            leftFront  = leftFront  * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack   = leftBack   * SF * driveSpeed;
            rightBack  = rightBack  * SF * driveSpeed;


            // DRIVE!!!
            robot.drive.drive(leftFront, rightFront, leftBack, rightBack);


            /** Tail Control **/
            // Once we hit endgame and we haven't been overridden, tell Jack
            if ((elapsedGameTime.seconds() > 105.0) && !overrodeLiftTail) {
                liftingTail = true;
            }
            // Exit the auto lift Tail if the driver touches it
            if (liftingTail && ((Math.abs(gamepad1.left_trigger - gamepad1.right_trigger)) > 0.2)) {
                // Tell code that we overrode its DREAMS to avoid unwanted repeats!
                overrodeLiftTail = true;
                // Set the encoder back to normal for TeleOp
                robot.tail.tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            // Raise Tail during endgame automatically
            if (liftingTail && !overrodeLiftTail) {
                // Driver enhancement help for Tail during TeleOp
                robot.tail.lowerRobot();
                telemetry.addData("Tail Test", "After extend");
            } else {
                // Move normally
                robot.tail.tailMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }





            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Intake Spinning Controls  (For whatever reason, 0.87 is the magic number)
            robot.arm.intakeServo.setPower(gamepad2.right_trigger*0.87 - gamepad2.left_trigger*0.87);

            //**  Arm controls **//

            /**
             * If the driver presses A, Jack enters his "auto-mode" where he
             * automatically rotates his arm to the correct position and will
             * extend at an optimal time to score minerals autonomously in TeleOp.
             */
            if (gamepad2.a){
                robot.arm.armMotor.setPower(-1);
                autoArm = true;
            }

            // Tests if it is in auto-mode
            if (autoArm){
                // Exits auto-mode if the driver tries to move the arm themselves
                if (Math.abs(gamepad2.right_stick_y) > 0.55) {
                    autoArm = false;
                }
                // Starts extending at the right time
                if (robot.arm.armMotor.getCurrentPosition() < CatMecanumHW.ARM_EXTEND) {
                    robot.extend.extenderMotor.setPower(-1.0);
                }
                // Slows down the arm's movement when closing in on the target to not overshoot
                if ((robot.arm.armMotor.getCurrentPosition() < CatMecanumHW.ARM_SLOW) && !slowArm) {
                    robot.arm.armMotor.setPower(-0.7);
                    slowArm = true;
                }
                // Stops arm movement once done and resets the variables
                if (robot.arm.armMotor.getCurrentPosition() < CatMecanumHW.ARM_TELEOP) {
                    robot.arm.armMotor.setPower(0.0);
                    autoArm = false;
                    slowArm = false;
                }
                //if the driver isn't in auto mode it uses these methods
            } else {
                /**
                 * Otherwise run the normal Driver 2 code
                 */

                // Lower and raise the arm by rotations
                robot.arm.armMotor.setPower(gamepad2.right_stick_y);

                // Extend and retract the arm
                robot.extend.extenderMotor.setPower(gamepad2.left_stick_y * 0.8);
            }


            // Open/Close gate
            if(gamepad2.left_bumper) {
                robot.arm.gateClose();
            } else if (gamepad2.right_bumper) {
                robot.arm.gateOpen();
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
            telemetry.addData("Tail Encoder Position:", robot.tail.tailMotor.getCurrentPosition());
            telemetry.addData("Intake Speed:", robot.arm.intakeServo.getPower());
            telemetry.update();
        }
    }
}
