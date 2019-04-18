/**
 ResetTail.java

 A Linear OpMode class that has one simple task: Retract
 the tail so that Jack will be within the 18 inch sizing
 cube.  It does this by checking the encoder values, and
 when they stop changing, the code assumes the tail is
 inside the robot and shuts down the motor.  This is a
 simple autonomous we can run before all of our matches
 instead of loading up the TeleOp and manually setting
 the tail.

 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Reset Tail", group="CatAuto")
public class ResetTail extends LinearOpMode {

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
                robot.tail.autoResetTail();
                delayTimer.reset();
            }
        }
        /**
         * This used to be an entire autonomous routine that would reset
         * the tail after play until we decided to put it into a method
         * so that we could call it during the autonomous runs using a
         * separate thread.
         */
    }
}
