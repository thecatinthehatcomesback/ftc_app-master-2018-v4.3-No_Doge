/***
    MecAutonomousLevel5_ScoreMinerals.java

 A Linear OpMode class used as an autonomous routine for both the Blue & Red
 sides of the field.  During init, where we can select which side of the lander
 the robot thinks it is hanging off of at the start of the match and detect the
 gold with TensorFlow.  After "Play" is pressed, Jack grabs the correct element
 with the intake, places the Cat Hat team marker into our depot, and extends the
 arm into either crater to count as parking according to what we chose during
 init using gamepad1 before the start of the match.  This level will score
 minerals using whatever time remains after the match.  If we run the Crater side,
 we skip dropping off the marker to spend all our time grabbing and scoring
 minerals since 4 minerals is more than one team marker drop off.

 MecAutonomousLevel5_ScoreMinerals was written for the Detroit Worlds FIRST Tech
 Challenge competition.  It accounts for the iterations and modifications added
 to our robot since the MN State Tournament.  Some of these include an improved
 arm and intake.

 MecAutonomousLevel5_ScoreMinerals uses the CatAsyncHW class in order to
 run operate multiple bits of code at once to save time.  Thus, giving
 more time to grab and score minerals during the autonomous.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Score Minerals Autonomous", group="CatAuto")
public class MecAutonomousLevel5_ScoreMinerals extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
    CatVisionHW eyes = new CatVisionHW();     // Doge and vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = false;
    private boolean parkInOurCrater = false;
    private boolean closeToWall = true;

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
        // Init our Machine Vision right away
        eyes.initVision(hardwareMap);

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
         * After init is pushed but before Start we
         * can change presets in the OpMode using
         * the gamepad buttons.
         */

        // Reset the timer to avoid any unwanted numbers
        delayTimer.reset();
        // Runs a loop to change the presets while we wait to start
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
            // Changes which alliance color we belong to
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

            // Change which crater we'll park in
            if (((gamepad1.a) && delayTimer.seconds() > 0.8)) {
                if (parkInOurCrater) {
                    parkInOurCrater = false;
                } else {
                    parkInOurCrater = true;
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

            // Change how close to the wall we go
            if (((gamepad1.y) && delayTimer.seconds() > 0.8)) {
                if (closeToWall) {
                    closeToWall= false;
                } else {
                    closeToWall = true;
                }
                delayTimer.reset();
            }

            if (Math.abs(robot.drive.getCurrentAngle())>1){
                robot.drive.IMUinit();
                Log.d("catbot", String.format("IMU re-initialized"));
            }




            // Cool LED code:
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
                telemetry.addData("Alliance:", "Red");
            } else {
                telemetry.addData("Alliance:", "Blue");
            }

            if (isCraterSide) {
                telemetry.addData("Lander Side:", "Crater");
            } else {
                telemetry.addData("Lander Side:", "Depot");
            }

            if (parkInOurCrater) {
                telemetry.addData("Parking in the Crater:", "On our alliance side...");
            } else {
                telemetry.addData("Parking in the Crater:", "Of the other alliance's side...");
            }
            if (closeToWall) {
                telemetry.addData("End position is:", "Close to the wall");
            } else {
                telemetry.addData("End position is:", "Away from wall");
            }

            /**
             * Find and store the values of the sampling right away
             * while we are hanging to maximize our camera's PoV
             * (Point of View)
             */
            eyes.findGoldPos();
            // Tell driver which is seen
            telemetry.addData("Find Gold:", eyes.samplingValues.getLast());
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

        Log.d("catbot", String.format(" Hit Start"));

        // Close down the vision to reduce RAM usage
        eyes.tfod.deactivate();
        Log.d("catbot", String.format("Tensor Flow deactivated"));

        // Give the sampling position
        samplingPos = eyes.giveSamplePos();
        /// TODO: 3/4/2019 GET THE AMOUNT OF LOOPS IT ACTUALLY HAPPENS
        telemetry.addData("LoopCount:", eyes.loopCount);
        telemetry.update();




        // Lower the robot and begin!
        robot.tail.lowerRobot();
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.tail.waitUntillDone();
        // Drive the robot out the hook
        robot.drive.mecDriveHorizontal(robot.DRIVE_SPEED,4.5,2.0);
        robot.drive.waitUntillDone();
        robot.drive.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(.4,2,1);
        robot.drive.waitUntillDone();

        robot.tail.resetTail();
        robot.spawnWait(robot.tail);

        // LED feedback for the sampling field
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

        // Delay the rest of the autonomous for the number of seconds we chose
        robot.robotWait(timeDelay);

        // Enter the rest of the autonomous based on which side we selected near beginning
        if (isCraterSide) {
            driveCrater();
        }else {
            driveDepot();
        }

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
    //recenters to 0 degrees
        robot.drive.mecTurn(.4,2,1);
        robot.drive.waitUntillDone();
        switch (samplingPos) {
            case LEFT:
                robot.drive.mecTurn(robot.TURN_SPEED,-12,2.5);
                break;
            case RIGHT:
                robot.drive.mecTurn(robot.TURN_SPEED,15,2.5);
                break;
        }

        //Pick up gold
        robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
        //wait untill both the arm is lowered and the has driven to the correct position to sample
        CatSubsystemHW.waitUntillDone(robot.arm, robot.drive);
        robot.extend.extendArm();
        robot.extend.waitUntillDone();
        robot.extend.retractArm();
        robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);


        //drives forward to avoid hitting the lander's leg
        robot.robotWait(.5);
        robot.drive.mecTurn(CatMecanumHW.TURN_SPEED,-26,3);
        CatSubsystemHW.waitUntillDone(robot.arm, robot.extend, robot.drive);
        robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,10,3.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-84,3.5);
        robot.drive.waitUntillDone();
        //drives toward wall and aims toward the crater
        robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,27,6,CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(CatMecanumHW.TURN_SPEED,-110,2);
        robot.drive.waitUntillDone();
        //lower arm
        robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        //Extend arm to depot
        robot.extend.extendArm();
        CatSubsystemHW.waitUntillDone(robot.arm,robot.extend);
        //Spit out team marker
        robot.arm.intakeServo.setPower(-0.87);
        robot.robotWait(.5);
        //Bring back arm
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.arm.waitUntillDone();
        robot.arm.intakeServo.setPower(0.0);
        robot.extend.retractArm();
        robot.extend.waitUntillDone();
        //lifts arm to avoid hitting the lander's leg
        robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
        robot.arm.waitUntillDone();
        //turns to aim at the sample field
        robot.drive.mecTurn(0.9,12,3);
        robot.drive.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.extend.extendArm();
        CatSubsystemHW.waitUntillDone(robot.arm,robot.extend);
        /**robot.intakeServo.setPower(-.87);
        //tries to sample from the side
        switch (samplingPos) {
            case LEFT:
                break;
            case CENTER:
                robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,-7,2.5,CatMecanumHW.DRIVE_MODE.driveTilDistance);
                robot.extendArm();
                break;
            case RIGHT:
                robot.extendArm();
                robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,5,2.5,CatMecanumHW.DRIVE_MODE.driveTilDistance);
                break;
            case UNKNOWN:
                break;
        }
        robot.mecTurn(robot.TURN_SPEED, 77, 1.5);
        robot.robotWait(.3);
        robot.mecTurn(CatMecanumHW.TURN_SPEED,90,1.5);
        robot.intakeServo.setPower(0.0);
        //puts the arm into the crater
        robot.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        if (samplingPos==CatVisionHW.samplingPos.LEFT) {
            robot.extendArm();
        }
        robot.mecTurn(CatMecanumHW.TURN_SPEED,25,2.5);
        robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        //tries to pick up minerals
        robot.intakeServo.setPower(.87);
        robot.robotWait(.75);
        robot.intakeServo.setPower(0.0);
*/

    }
    public void driveDepot() throws InterruptedException {

        /**
         * Depot/marker dropoff code
         */
        //Extend arm to depot
        Log.d("catbot", String.format(" Start extending to depot"));
        robot.extend.extendArm();
        // Drive ahead to deploy marker
        robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 10, 2, CatDriveHW.DRIVE_MODE.driveTilDistance);
        CatSubsystemHW.waitUntillDone(robot.arm, robot.drive, robot.extend);
        //Finish lower arm
        robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        //Spit out team marker
        robot.arm.intakeServo.setPower(-0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(0.25);
        //Bring back arm
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.arm.intakeServo.setPower(0.0);
        // Pull arm in
        robot.extend.retractArm();
        CatSubsystemHW.waitUntillDone(robot.extend, robot.arm);

        /**
         * Pick up gold mineral and drop it off in the lander
         */
        // Drive back to hit gold
        Log.d("catbot", String.format(" back up to hit gold"));
        robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, -10, 4, CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
        //Turn to gold correct gold position
        switch (samplingPos) {
            case LEFT:
                robot.drive.mecTurn(robot.TURN_SPEED,-19,2.5);
                break;
            case RIGHT:
                robot.drive.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        CatSubsystemHW.waitUntillDone(robot.drive, robot.arm);
        //Extend to pick up the gold
        robot.arm.intakeServo.setPower(0.87);
        robot.extend.extendArm();
        robot.extend.waitUntillDone();
        // Pull the arm back in and rotate the arm to the scoring position
        Log.d("catbot", String.format(" pull back in and rotate it to score"));
        robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
        robot.extend.retractArm();
        robot.arm.intakeServo.setPower(0.0);
        //drives back to the lander at the same time the arm rotates.
        robot.drive.mecTurn(.35,0,2);
        //This will extend arm so that it reaches full extension and rotation at similar times.
        robot.arm.setExtenderValue(3000);
        CatSubsystemHW.waitUntillDone(robot.drive,robot.arm);
        robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
        //finishes driving to the lander
        robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,-9,3, CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        //opens the gate and scores the mineral, jiggling it out of the intake
        robot.arm.gateOpen();
        robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.65);
        //robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN,.8);
        //robot.robotWait(.1);
        //robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.65);
        robot.arm.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
        robot.arm.waitUntillDone();
        //starts to drive away from the lander and to the crater
        robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,3,2, CatDriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        //Once far enough from lander we safely withdraw and rotate arm to crater height
        robot.extend.retractArm();
        robot.extend.waitUntillDone();
        robot.arm.gateClose();


        //sets LEDs to color of the alliance
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }


        /// TODO: 2/4/2019 Maybe a center versus wall option so that we don't hit alliance partner
        if (parkInOurCrater) {
            // Drive to crater nearest the audience
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,27,3.5);
            CatSubsystemHW.waitUntillDone(robot.drive,robot.extend);
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,12,3.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            if (closeToWall)
            {
                robot.drive.mecTurn(CatMecanumHW.TURN_SPEED, 73, 3.5);
                robot.drive.waitUntillDone();
                //Lower arm
                robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 32, 3.0, CatDriveHW.DRIVE_MODE.driveTilDistance);
                robot.drive.waitUntillDone();
                robot.drive.mecTurn(CatMecanumHW.TURN_SPEED, 110, 1);
                robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            }
            else {
                robot.drive.mecTurn(CatMecanumHW.TURN_SPEED, 78, 3.5);
                robot.drive.waitUntillDone();
                //Lower arm
                robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
                robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 14, 3.0, CatDriveHW.DRIVE_MODE.driveTilDistance);
                robot.drive.waitUntillDone();
                robot.drive.mecTurn(CatMecanumHW.TURN_SPEED, 120, 1);
                robot.drive.waitUntillDone();
                robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 6, 3.0, CatDriveHW.DRIVE_MODE.driveTilDistance);
            }
        } else {
            // Drive to farther crater
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-27,3.5);
            CatSubsystemHW.waitUntillDone(robot.drive,robot.extend);
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,14,3.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-85,3.5);
            //Lower arm
            robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            CatSubsystemHW.waitUntillDone(robot.drive,robot.arm);
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,14,3.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
            //robot.drive.waitUntillDone();
            //robot.drive.mecTurn(CatMecanumHW.TURN_SPEED,-100,1);
            robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        }

        //Extend arm to crater
        CatSubsystemHW.waitUntillDone(robot.drive,robot.arm);
        robot.extend.extendArm();
        robot.robotWait(.4);
        //tries to pick up minerals from the crater
        robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
        robot.arm.intakeServo.setPower(0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(1.0);

        //attempts to score minerals if we are set to go to the opposing alliance's crater
        ///TODO: 3/13/2019 test to see if we have enough time to drop off and get back to the crater also get it to work with both craters
        if (!parkInOurCrater) {
            //starts bring the arm back in and rotates it to the scoring position
            robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
            robot.extend.retractArm();
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-90,3.5);
            //drives back toward the lander
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, -14, 3.0, CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.arm.intakeServo.setPower(0.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-50,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,-18,3.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            CatSubsystemHW.waitUntillDone(robot.extend, robot.arm);
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-9,2);
            robot.robotWait(.3);
            //recenters to the lander
            robot.drive.mecTurn(.4,-4,2);
            robot.drive.waitUntillDone();
            //drives into the lander
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,-3,2.0,CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
            robot.robotWait(.4);
            //opens the gate and tries to score the minerals by wiggling them out
            robot.arm.gateOpen();
            robot.drive.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.45);
            //robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN,.8);
            //robot.robotWait(.1);
            //robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.65);
            robot.arm.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
            robot.arm.waitUntillDone();
            //drives away from the lander and reenters the crater
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,9,2,CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            //resets the gate and lowers the arm while driving
            robot.arm.gateClose();
            robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            robot.drive.mecTurn(CatMecanumHW.TURN_SPEED ,-85,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 16, 2, CatDriveHW.DRIVE_MODE.driveTilDistance);
            robot.extend.extendArm();
            robot.drive.waitUntillDone();
            //Finishes with all autonomous goals completed and one extra cycle

        } else {
            robot.arm.intakeServo.setPower(0.0);
        }


    }
}
