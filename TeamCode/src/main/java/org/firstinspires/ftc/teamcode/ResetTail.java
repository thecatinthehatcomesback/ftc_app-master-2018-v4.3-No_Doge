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


 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Reset Tail", group="CatAuto")
public class ResetTail extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
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

        // Run the method
        robot.tail.resetTail();

    }
}
