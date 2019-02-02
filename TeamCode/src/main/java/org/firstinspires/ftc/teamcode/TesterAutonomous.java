/**
 TesterAutonomous.java

 A Linear OpMode class to be place to test code
 both old and new.  We constantly edit this, taking
 out and adding in code.  This is never the same.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Testing Autonomous", group="CatAuto")
public class TesterAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHardware robot = new CatMecanumHardware();   // Use the mecanum hardware
    CatVisionHardware eyes = new CatVisionHardware();   // Use the mecanum hardware
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
        //eyes.initDogeforia(hardwareMap, this);

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

        double intakeTIMING = 0;

        /*robot.advMecDrive(CatMecanumHardware.DRIVE_SPEED, 20, 0+45, 3);
        robot.advMecDrive(CatMecanumHardware.DRIVE_SPEED, 20, 90+45, 3);
        robot.advMecDrive(CatMecanumHardware.DRIVE_SPEED, 20, 180+45, 3);
        robot.advMecDrive(CatMecanumHardware.DRIVE_SPEED, 20, 270+45, 3);*/



        /**
         * What we plan to test in this autonomous:
         *
         *
         */
    }
}
