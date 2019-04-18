/*
        TestTeleOp.java

    A Linear opMode class to be our TeleOp testing method to try
    and solve our problems throughout the year without having to
    modify the main TeleOp.

*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    CatAsyncHW robot; // use the class created for the hardware
    //CatVisionHW eyes = new CatVisionHW();   // Use the mecanum hardware
    boolean inReverse = true;

    // constructor for class
    public TestTeleOp() {
        robot = new CatAsyncHW();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        robot.drive.IMUinit();

        // Init our Machine Vision
        //eyes.initVision(hardwareMap);

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
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean slowArm = false;
        boolean autoArm = false;
        boolean liftingTail = false;
        boolean overrodeLiftTail = false;

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





            // Reset Arm
            if (gamepad1.x) {
                robot.arm.autoResetArm();
            }
            // IMU Sensor
            Orientation angles = robot.drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            // Tail Motor Pos
            //telemetry.addData("Tail Motor Position: ", robot.tail.tailMotor.getCurrentPosition());
            telemetry.addData("Arm Rotate Encoder # and Boolean","%d,  %b",robot.arm.armMotor.getCurrentPosition(), robot.arm.isRotateLimit());
            telemetry.addData("Arm Extend Encoder","%d",robot.extend.extenderMotor.getCurrentPosition());
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
            // LED Telemetry
            telemetry.addData("Pattern", "%s",robot.pattern.toString());

            telemetry.update();
        }
    }
}
