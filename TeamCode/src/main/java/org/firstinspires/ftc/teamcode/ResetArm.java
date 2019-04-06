package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ResetArm", group="CatTeleOp")


public class ResetArm extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatMecanumHW robot; // use the class created for the hardware
    CatVisionHW eyes = new CatVisionHW();   // Use the mecanum hardware

    // constructor for class
    public ResetArm() {
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

        if (robot.isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }
        // Go!
        runTime.reset();
        elapsedGameTime.reset();

        boolean resetMode = false;
        boolean hasReset = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            if (gamepad2.y) {
                if (robot.armLimit.getState()) {
                    robot.armMotor.setPower(-0.70);
                } else {
                    robot.armMotor.setPower(-0.55);
                    hasReset = true;
                }

                resetMode = true;
            }

            if (resetMode) {

                if (!hasReset) {
                    if (!robot.armLimit.getState()) {
                        robot.armMotor.setPower(0.40);
                        hasReset = true;
                    }
                } else {
                    if (robot.armLimit.getState()) {
                        robot.armMotor.setPower(0.00);
                        resetMode = false;
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.robotWait(0.5);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setTargetPosition(-825);
                        robot.armMotor.setPower(0.50);
                        while (robot.armMotor.isBusy()){ }
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    }
                }
            } else {
                    //**  Arm controls **//
                    // Lower/Raise arm
                    robot.armMotor.setPower(gamepad2.right_stick_y);
                    // Extend/Retract arm
                    robot.extenderMotor.setPower(gamepad2.left_stick_x * 0.8);
            }
            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */

            telemetry.addData("Arm Encoder", "%d,  %b", robot.armMotor.getCurrentPosition(), robot.armLimit.getState());
            telemetry.addData("Extend Encoder", "%d", robot.extenderMotor.getCurrentPosition());
            telemetry.update();


        }
    }
}
