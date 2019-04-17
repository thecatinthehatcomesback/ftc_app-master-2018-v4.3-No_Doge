/***
    TesterAutonomous.java

 A Linear OpMode class to be place to test code both old
 and new.  We constantly edit this, taking out and adding
 in code.  This is never the same at any given time.

 This file uses the old CatMecanumHW.java file...

 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test Auto", group="CatAuto")
public class TesterAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new async mecanum hardware
    private ElapsedTime delayTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */

        robot.init(hardwareMap, this);
        // Init IMU sensor later when the match starts

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */
        robot.drive.IMUinit();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status:", "Initializing: (Resetting Encoders...)");
        telemetry.update();

        robot.drive.resetEncoders();
        idle();
        robot.drive.runToPosition();

        /**
         * After init is pushed but before Start.
         */


        waitForStart();
        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
          */

        Log.d("catbot", String.format(" Hit Start"));

        robot.drive.mecTurn(CatDriveHW.TURN_SPEED, -90, 3);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(CatDriveHW.TURN_SPEED, 90, 2, CatDriveHW.TURN_MODE.TANK);
        robot.drive.waitUntillDone();

//robot.robotWait();
        /*robot.arm.rotateArm(robot.ARM_OVER_SAMPLING);
        robot.arm.waitUntillDone();
        robot.extend.extendArm();
        robot.robotWait(0.4);
        //tries to pick up minerals from the crater
        robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
        robot.arm.intakeServo.setPower(0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(1.0);

        robot.arm.rotateArm(robot.ARM_STRAIGHT_UP);
        robot.robotWait(6.0);
        robot.arm.rotateArm(robot.ARM_STOWED);*/

    }
}
