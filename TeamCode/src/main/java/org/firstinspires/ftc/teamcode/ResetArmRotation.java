/**
 ResetArmRotation.java

 A Linear OpMode class that has one simple task: Rotate
 the arm a few times while resetting the encoders so that
 Jack will be within the 18 inch sizing cube and enhance
 autonomous accuracy.  It does this by checking the
 armRotateLimit switch, and rotating the arm accordingly.
 Once it reaches the limit, it resets the encoders, moves
 several ticks back and resets the encoders again.  This
 is a simple autonomous we can run before all of our
 matches instead of loading up the TeleOp and manually
 setting the arm that could end in multiple human errors.


 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Reset Arm", group="CatAuto")
public class ResetArmRotation extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
    private ElapsedTime delayTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry message to signify robot is waiting
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();

        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */
        robot.init(hardwareMap, this);

        // Once init finished
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        waitForStart();
        /**
         * Runs after hit start
         * DO STUFF FOR MODE!!!!!!!!!!!
         *
         \*/
        delayTimer.reset();
        telemetry.addData("Press", "gamepad1 X");
        telemetry.update();

        while (opModeIsActive()) {
            // Auto reset the arm
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                robot.arm.autoResetArm();
                delayTimer.reset();
            }
        }
        /**
         * This used to be an entire autonomous routine that would reset
         * the arm after play until we decided to put it into a method
         * so that we could call it during the autonomous inits using a
         * separate thread.
         */
    }
}
/*
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
                    //  Arm controls
        // Lower/Raise arm
        robot.armMotor.setPower(gamepad2.right_stick_y);
        // Extend/Retract arm
        robot.extenderMotor.setPower(gamepad2.left_stick_x * 0.8);
        }
 */