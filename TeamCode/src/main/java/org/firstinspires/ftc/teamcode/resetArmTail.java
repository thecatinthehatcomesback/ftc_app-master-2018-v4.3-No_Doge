/**
 resetArmTail.java

 A Linear OpMode class that has two simple tasks: Reset
 the arm and tail and reset both encoders so that
 Jack will be within the 18 inch sizing cube and enhance
 autonomous accuracy.  It does this by checking the
 armRotateLimit switch, and rotating the arm accordingly.
 Once it reaches the limit, it resets the encoders, moves
 several ticks back and resets the encoders again.  This
 is a simple autonomous we can run before all of our
 matches instead of loading up the TeleOp and manually
 setting the arm that could end in multiple human errors.

 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Reset Arm & Tail", group="CatAuto")
public class resetArmTail extends LinearOpMode {

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
        telemetry.addData("Press PLAY", "to reset the Arm and Tail");
        telemetry.update();

        // Auto reset the tail and arm
        robot.tail.autoResetTail();
        robot.spawnWait(robot.tail);
        robot.arm.autoResetArm();
    }
}
