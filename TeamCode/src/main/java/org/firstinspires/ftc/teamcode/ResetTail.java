/**
 ResetTail.java

 A Linear OpMode class that has one simple
 task.  Retract the tail so that it will
 always be within the 18 inch cube our
 robot.  This is a simple autonomous we
 run before all of our matches instead of
 always having to load up the TeleOp and
 manually setting the tail.


 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Reset Tail", group="CatAuto")
public class ResetTail extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHW robot = new CatMecanumHW();   // Use the mecanum hardware
    private ElapsedTime runtime = new ElapsedTime();

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
        int newEncTicks;
        int oldEncTicks = 40;

        runtime.reset();
        robot.tailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            // Get the current encoder value
            newEncTicks = robot.tailMotor.getCurrentPosition();
            // Set tail to retract slowly at quarter power
            robot.tailMotor.setPower(-0.4);

            // Every so often, check the values
            if (runtime.milliseconds() > 750) {
                // Once the encoder ticks slows down or stops
                if ((newEncTicks - oldEncTicks) > -80) {
                    // Cut power to tail
                    robot.tailMotor.setPower(0.0);
                    // Tell EVERYONE!!!
                    telemetry.addData("Status: ", "Success!!!");
                    telemetry.update();
                    robot.robotWait(3.0);
                    // End OpMode
                    stop();
                }

                // Otherwise continue as normal and reset timer
                telemetry.addData("Status: ", "Still going...");
                runtime.reset();
                oldEncTicks = newEncTicks;
            }
            telemetry.addData("Current Enc Ticks: ", newEncTicks);
            telemetry.addData("Old Enc Ticks: ", oldEncTicks);
            telemetry.update();
        }

        /**
         * Keep the motor running at a low power until the
         * tail is completely inside the robot.  Have the
         * robot watch the encoder values and stop the motor
         * after the encoder values change is under a certain
         * amount of ticks per half second.
         */
    }
}
