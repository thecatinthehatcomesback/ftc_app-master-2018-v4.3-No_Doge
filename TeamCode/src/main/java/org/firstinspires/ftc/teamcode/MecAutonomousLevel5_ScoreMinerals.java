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



*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.CatVisionHW.samplingPos.LEFT;


@Autonomous(name="Score Minerals Autonomous", group="CatAuto")
public class MecAutonomousLevel5_ScoreMinerals extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
    CatVisionHW eyes = new CatVisionHW();     // Doge and vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = true;
    private boolean parkInOurCrater = false;
    private boolean teamMarker = false;

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

            // Change If we go to drop off the team marker or not
            if (((gamepad1.b) && delayTimer.seconds() > 0.8)) {
                if (teamMarker) {
                    teamMarker = false;
                } else {
                    teamMarker = true;
                }
                delayTimer.reset();
            }

            // Re-Init the IMU if it starts to drift
            if (Math.abs(robot.drive.getCurrentAngle()) > 1){
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

            /* Show the Telemetry: */
            telemetry.addData("Delay Timer:", timeDelay);

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
            if (teamMarker){
                telemetry.addData("Team marker is:", "Dropped off");
            } else {
                telemetry.addData("Team marker is:", "Not used");
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
        telemetry.addData("LoopCount:", eyes.loopCount);
        telemetry.update();




        // Lower the robot and begin!
        robot.tail.lowerRobot();
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.tail.waitUntillDone();
        // Drive the robot out the hook
        robot.drive.mecDriveHorizontal(robot.HYPER_SPEED,4.5,2.0);
        robot.drive.waitUntillDone();
        robot.drive.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(.5,0,1);
        robot.drive.waitUntillDone();

        robot.tail.autoResetTail();
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

           //aims to the correct gold location
           switch (samplingPos) {
               case LEFT:
                   robot.drive.mecTurn(.35, -18, 2.5);
                   break;
               case RIGHT:
                   robot.drive.mecTurn(CatDriveHW.TURN_SPEED, 17, 2.5);
                   break;
               case CENTER:
                   robot.drive.mecTurn(CatDriveHW.CREEP_SPEED, 7, 1);
                   break;
           }

           //Pick up gold
           robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
           //wait untill both the arm is lowered and the has driven to the correct position to sample
           CatSubsystemHW.waitUntillDone(robot.arm, robot.drive);

        if (teamMarker) {
           robot.extend.extendArm();
           robot.extend.waitUntillDone();
           robot.extend.retractArm();
           robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);


           //drives forward to avoid hitting the landers leg
           robot.robotWait(.5);
           robot.drive.mecTurn(CatDriveHW.TURN_SPEED, -26, 3);
           CatSubsystemHW.waitUntillDone(robot.arm, robot.extend, robot.drive);
           robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 10, 3.0);
           robot.drive.waitUntillDone();
           robot.drive.mecTurn(CatDriveHW.TURN_SPEED, -84, 3.5);
           robot.drive.waitUntillDone();
           //drives toward wall and aims toward the craytor
           robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 27, 6);
           robot.drive.waitUntillDone();
           robot.drive.mecTurn(CatDriveHW.TURN_SPEED, -110, 2);
           robot.drive.waitUntillDone();
           //lower arm
           robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
           //Extend arm to depot
           robot.extend.extendArm();
           CatSubsystemHW.waitUntillDone(robot.arm, robot.extend);
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
           robot.drive.mecTurn(0.9, 12, 3);
           robot.drive.waitUntillDone();
           robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
           robot.extend.extendArm();
           CatSubsystemHW.waitUntillDone(robot.arm, robot.extend);
        } else {
            //No team marker process
            robot.arm.intakeServo.setPower(0.87);
            robot.extend.extendEncoder(CatExtendHW.EXTEND_CRATER);
            robot.extend.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
            robot.extend.retractArm();
            //drives back to the lander at the same time the arm rotates.
            if (samplingPos == LEFT){
                robot.drive.mecTurn(CatDriveHW.CHILL_SPEED, 8, 2);
            }else {
                robot.drive.mecTurn(CatDriveHW.CHILL_SPEED,10,2);
            }
            //This will extend arm so that it reaches full extension and rotation at similar times.
            robot.arm.setExtenderValue(3000);
            CatSubsystemHW.waitUntillDone(robot.drive, robot.arm);
            robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
            robot.arm.intakeServo.setPower(0.0);
            robot.arm.gateOpen();
            robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.43);
            robot.arm.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
            robot.arm.waitUntillDone();
            //starts to drive away from the lander and to the crater
            //Once far enough from lander we safely withdraw and rotate arm to crater height
            //robot.extend.retractArm();
            robot.arm.gateClose();


            for (int i = 0; i < 2; i++) {
                // Lower and extend the arm
                robot.arm.rotateArm(CatMecanumHW.ARM_CRATER_GRAB);
                robot.extend.extendEncoder(CatExtendHW.EXTEND_OUT);
                robot.arm.intakeServo.setPower(0.87);
                robot.arm.waitUntillDone();
                //CatSubsystemHW.waitUntillDone(robot.arm, robot.extend);

                // Drive ahead and pick up
                robot.drive.mecDriveVertical(0.29, 13, 2);
                robot.robotWait(.12);
                robot.extend.extendArm();
                robot.drive.waitUntillDone();
                // does an extra pump
                robot.extend.retractArm();
                robot.robotWait(.08);
                robot.extend.extendArm();
                robot.robotWait(.08);
                // Bring arm up and in but spit out any extra block
                robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
                robot.arm.waitUntillDone();
                robot.arm.intakeServo.setPower(-0.87);
                robot.robotWait(0.1);
                robot.arm.intakeServo.setPower(0.0);
                robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
                robot.extend.retractArm();

                robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, -11, 1.5);
                robot.drive.waitUntillDone();
                robot.drive.mecTurn(CatDriveHW.CHILL_SPEED, 8, 2);
                CatSubsystemHW.waitUntillDone(robot.drive, robot.arm);

                robot.arm.setExtenderValue(3300);
                robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
                robot.robotWait(0.35);
                // Open gate and score minerals
                robot.arm.gateOpen();
                robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.43);
                robot.arm.waitUntillDone();
                robot.robotWait(0.3);
                robot.drive.mecTurn(CatDriveHW.CHILL_SPEED, 0, 2);
                robot.drive.waitUntillDone();
                robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
                robot.arm.waitUntillDone();
                robot.arm.gateClose();
            }

            // Get into the crater now
            robot.arm.rotateArm(CatMecanumHW.ARM_CRATER_SCORE);
            robot.extend.extendArm();
            CatSubsystemHW.waitUntillDone(robot.arm, robot.extend);
        }

    }
    public void driveDepot() throws InterruptedException {

        /**
         * Depot/marker dropoff code
         */
        //Extend arm to depot
        Log.d("catbot", String.format(" Start extending to depot"));
        robot.extend.extendArm();
        // Drive ahead to deploy marker
        robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 10, 2);
        CatSubsystemHW.waitUntillDone(robot.arm, robot.drive, robot.extend);
        //Finish lower arm
        robot.arm.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        //Spit out team marker
        robot.arm.intakeServo.setPower(-0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(0.25);
        //Bring back arm
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        // Pull arm in
        robot.extend.retractArm();
        CatSubsystemHW.waitUntillDone(robot.extend, robot.arm);
        robot.arm.intakeServo.setPower(0.0);

        /**
         * Pick up gold mineral and drop it off in the lander
         */
        // Drive back to hit gold
        Log.d("catbot", String.format(" back up to hit gold"));
        robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, -10, 2);
        robot.drive.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHW.ARM_FLOOR);
        //Turn to gold correct gold position
        switch (samplingPos) {
            case LEFT:
                robot.drive.mecTurn(robot.TURN_SPEED,-16,2.5);
                break;
            case RIGHT:
                robot.drive.mecTurn(robot.TURN_SPEED,16,2.5);
                break;
            case CENTER:
                robot.drive.mecTurn(.35,8,2.5);
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
        CatSubsystemHW.waitUntillDone(robot.drive, robot.arm);
        robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
        //finishes driving to the lander
        robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,-2,3);
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
        //robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 4.5, 2, CatDriveHW.DRIVE_MODE.driveTilDistance);
        //robot.drive.waitUntillDone();
        //Once far enough from lander we safely withdraw and rotate arm to crater height
        robot.extend.retractArm();
        robot.arm.gateClose();


        //sets LEDs to color of the alliance
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }


        if (parkInOurCrater) {
            // Drive to crater nearest the audience
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,27,3.5);
            CatSubsystemHW.waitUntillDone(robot.drive, robot.extend);
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,12,3.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED, 78, 3.5);
            robot.drive.waitUntillDone();
            //Lower arm
            robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 14, 3.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED, 120, 1);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, 6, 3.0);
        } else {
            // Drive to farther crater
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,-27,3.5, CatDriveHW.TURN_MODE.TANK);
            CatSubsystemHW.waitUntillDone(robot.drive, robot.extend);
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,8,3.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,-87,3.5);
            //Lower arm
            robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            CatSubsystemHW.waitUntillDone(robot.drive,robot.arm);
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,14,3.0);
            //robot.drive.waitUntillDone();
            //robot.drive.mecTurn(CatDriveHW.TURN_SPEED,-100,1);
            robot.arm.rotateArm(CatMecanumHW.ARM_CRATER_GRAB);
        }

        //Extend arm to crater
        CatSubsystemHW.waitUntillDone(robot.drive,robot.arm);
        robot.arm.intakeServo.setPower(0.87);
        robot.extend.extendArm();
        robot.robotWait(.4);
        //tries to pick up minerals from the crater
        robot.arm.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.arm.waitUntillDone();
        robot.arm.intakeServo.setPower(-0.87);
        robot.robotWait(0.1);
        robot.arm.intakeServo.setPower(0.00);


        //attempts to score minerals if we are set to go to the opposing alliance's crater
        if (!parkInOurCrater) {
            //starts bring the arm back in and rotates it to the scoring position
            robot.arm.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
            robot.extend.retractArm();
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,-90,3.5);
            //drives back toward the lander
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED, -14, 3.0);
            robot.arm.intakeServo.setPower(0.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,-50,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,-18,3.0);
            robot.drive.waitUntillDone();
            CatSubsystemHW.waitUntillDone(robot.extend, robot.arm);
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED,-9,2);
            robot.robotWait(0.3);
            //recenters to the lander
            robot.drive.mecTurn(0.4,-4,2);
            robot.drive.waitUntillDone();
            //drives into the lander
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,-3,2.0);
            robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
            robot.robotWait(.4);
            //opens the gate and tries to score the minerals by wiggling them out
            robot.arm.gateOpen();
            robot.drive.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_SCORE,.45);
            robot.arm.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
            robot.arm.waitUntillDone();
            //drives away from the lander and reenters the crater
            robot.drive.mecDriveVertical(CatDriveHW.DRIVE_SPEED,12,2);
            robot.drive.waitUntillDone();
            //resets the gate and lowers the arm while driving
            robot.arm.gateClose();
            robot.arm.rotateArm(CatMecanumHW.ARM_CRATER_SCORE);
            robot.drive.mecTurn(CatDriveHW.TURN_SPEED ,-83,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatDriveHW.HYPER_SPEED, 20, 2);
            robot.extend.extendArm();
            robot.drive.waitUntillDone();
            //Finishes with all autonomous goals completed and one extra cycle

        }
    }
}
