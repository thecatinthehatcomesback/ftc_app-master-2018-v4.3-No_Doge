/**
 MecAutonomousLevel2_Intermediate.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of with gamepad1 and
 detect the gold with TensorFlow, hit the right element, place our team marker
 in our depot and park in either crater according to what we say using gamepad1
 at the beginning of the match.

 MecAutonomousLevel2_Intermediate is a more advanced version of the MecBasic we started
 with.  This autonomous is used for our 2nd qualifier this year (Dec 8, 2018).



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Autonomous(name="Intermediate Autonomous", group="CatAuto")
public class MecAutonomousLevel2_Intermediate extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHW robot = new CatMecanumHW();      // Use our mecanum hardware
    CatVisionHW eyes = new CatVisionHW();   // Doge and vision init
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = true;
    private boolean isParkNearCrater = true;

    private CatVisionHW.samplingPos samplingPos = CatVisionHW.samplingPos.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */
        robot.init(hardwareMap, this);
        // Init IMU sensor later when the match starts
        // Init our Machine Vision
        eyes.initVision(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :%7d  :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.leftRearMotor.getCurrentPosition(),
                robot.rightRearMotor.getCurrentPosition());
        telemetry.update();

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() ) {
            if (this.isStopRequested()) {
                return;
            }
            // Increases the amount of time we wait
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                timeDelay += 1;
                delayTimer.reset();
            }
            // Decreases the amount of time we wait
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                if (timeDelay > 0) {
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            // Changes which alliance color we are
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }

            //change which crater we'll park in :)
            if (((gamepad1.a) && delayTimer.seconds() > 0.8)) {
                if (isParkNearCrater) {
                    isParkNearCrater = false;
                } else {
                    isParkNearCrater = true;
                }
                delayTimer.reset();
            }

            // Changes which side of the lander our robot starts at
            if (((gamepad1.dpad_right) && delayTimer.seconds() > 0.8)) {
                if (isCraterSide) {
                    isCraterSide = false;
                } else {
                    isCraterSide = true;
                }
                delayTimer.reset();
            }

            // LED code...
            if (isRedAlliance) {
                if(isCraterSide){
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                 } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                }
            } else {
                if(isCraterSide){
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                }
            }

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            if (isCraterSide) {
                telemetry.addData("Lander Side: ", "Crater");
            } else {
                telemetry.addData("Lander Side: ", "Depot");
            }

            if (isParkNearCrater) {
                telemetry.addData("Parking Crater: ", "Near");
            } else {
                telemetry.addData("Parking Crater: ", "Far");
            }

            /**
             * Find and store the values of the sampling right away
             * while we are hanging to maximize our camera's PoV
             * (Point of View)
             */
            eyes.findGoldPos();
            // Tell driver which is seen...
            telemetry.addData("Find Gold:", samplingPos);
            telemetry.update();


            /**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }
        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */


        // Close down the vision to reduce RAM usage
        eyes.tfod.deactivate();
        // Give the sampling position
        samplingPos = eyes.giveSamplePos();

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */
        robot.IMUinit();

        /**
         * Runs after hit start
         * DO STUFF FOR OPMODE!!!!!!!!!!!
         *
         \*/


        // Lower robot here
        robot.lowerRobot();
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,3.0,2.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,-3.0,2.0);

        switch(samplingPos) {
            case LEFT:
                if (isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                }

                break;
            case RIGHT:
                if (isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                }

                break;
            case CENTER:
                if (isRedAlliance) {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
                } else {
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
                }

                break;
        }
        //Delay the amount we selected
        robot.robotWait(timeDelay);

        // Enter the rest of the autonomous based on which side we selected near beginning
        if (isCraterSide) {
            driveCrater();
        } else {
            driveDepot();
        }

        robot.retractTail();

        /**
         * What we plan to do in this autonomous:

         * Lower from the lander
         * Sample
         * Drop of team marker
         * Go to a Crater we choose at beginning
         * Crawl a little more up the side to make sure we don't slide
         */
    }
    public void driveCrater()  throws InterruptedException {
        // Slide if left or right
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 10, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, 14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, -20, 4.0);
                break;
        }
        // Drive forward
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 12, 4.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, -10.5, 4.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        // Switch back to the center
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, -14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, 20, 4.0);
                break;
        }
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }

        robot.mecTurn(robot.TURN_SPEED, -75, 3.0);

        // Drive Forward about 4 foot (To wall)
        robot.mecDriveVertical(robot.DRIVE_SPEED, 49.0, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(robot.TURN_SPEED, -127, 3.0);
        // Slides right against the wall
        robot.mecDriveHorizontal(CatMecanumHW.HYPER_SPEED,-5,2);
        // Drive into depot
        robot.mecDriveVertical(robot.DRIVE_SPEED, 45, 3.5, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED,3, 2);
        //robot.markerRelease();
        robot.mecDriveHorizontal(CatMecanumHW.HYPER_SPEED,3, 2);
        robot.robotWait (0.7);
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,-10,3, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        //robot.markerIn();
        robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED,-6,2);
        // Turn 45 towards the right crater
        if (isParkNearCrater) {
            robot.mecTurn(robot.TURN_SPEED, -134, 3.0);
        } else {
            robot.mecTurn(robot.TURN_SPEED, -41, 3.0);
        }

        // Drive Backwards 6 feet (To crater)
        robot.mecDriveVertical(robot.DRIVE_SPEED, -64.0, 8.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.robotWait( 0.5);
        robot.mecDriveVertical(robot.CREEP_SPEED, -4,3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
    }
    public void driveDepot() throws InterruptedException {

        // Slide if left or right
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 12, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, 14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, -22, 4.0);
                break;
        }
        // Drive forward
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 32, 4.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);

        // Switch back to the center
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, -14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED, 22, 4.0);
                break;
        }

        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }
        // Drive 4 foot and drop mineral off
        robot.mecDriveVertical(robot.DRIVE_SPEED, 13, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);

        // Turn 45 towards the right crater
        if (isParkNearCrater) {
            robot.mecTurn(robot.TURN_SPEED, -42, 3.0);
        } else{
            robot.mecTurn(robot.TURN_SPEED, 42, 3.0);
        }
        // Drop Marker
        //robot.markerRelease();
        robot.robotWait(1.0);
        //robot.markerIn();
        // Drive Backwards 6 feet (To crater)
        robot.mecDriveVertical(robot.DRIVE_SPEED, -15.0, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecDriveHorizontal(CatMecanumHW.DRIVE_SPEED,-9,3.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED, -72.0, 8.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.robotWait( 0.5);
        robot.mecDriveVertical(robot.CREEP_SPEED, -4,3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
    }
}
