/**
 TesterAutonomous.java

 A Linear OpMode class to be place to test code both old
 and new.  We constantly edit this, taking out and adding
 in code.  This is never the same at any given time.

 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Testing Autonomous", group="CatAuto")
public class TesterAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHW robot = new CatMecanumHW();   // Use the mecanum hardware
    CatVisionHW eyes = new CatVisionHW();   // Use the mecanum hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */
        robot.init(hardwareMap, this);
        robot.IMUinit();
        eyes.initVision(hardwareMap);



        // Send telemetry message to signify robot is waiting
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :7d  :7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.leftRearMotor.getCurrentPosition(),
                robot.rightRearMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        /**
         * Runs after hit start
         * DO STUFF FOR MODE!!!!!!!!!!!
         *
         \*/


        /*robot.advMecDrive(CatMecanumHW.DRIVE_SPEED, 20, 0+45, 3);
        robot.advMecDrive(CatMecanumHW.DRIVE_SPEED, 20, 90+45, 3);
        robot.advMecDrive(CatMecanumHW.DRIVE_SPEED, 20, 180+45, 3);
        robot.advMecDrive(CatMecanumHW.DRIVE_SPEED, 20, 270+45, 3);*/


        int numTimes = 0;

        delayTimer.reset();
        while (delayTimer.seconds() < 3) {
            eyes.findGoldPos();
            numTimes++;
        }

        delayTimer.reset();
        while (delayTimer.seconds() < 4) {
            telemetry.addData("Num Times", numTimes);
            telemetry.update();
        }


        /**
         * What we plan to test in this autonomous:
         *
         *
         */
    }
}
