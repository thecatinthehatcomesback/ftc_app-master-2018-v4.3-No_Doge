/**
    MecAutonomousLevel3_State.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of at the start of the match,
 detect the gold with TensorFlow, hit the correct element, place the  Cat Hat
 team marker in our depot, and park in either crater according to what we chose
 using gamepad1 at the beginning of the match.

 MecStateAutonomous is written for the Minnesota State competition to account for the
 changes to our robot since the December tournament including a new arm and intake.
(February 8-9, 2019)



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */

/***
 MecAutonomousLevel3_State.java

 A Linear OpMode class used as an autonomous routine for both the Blue & Red
 sides of the field.  During init, where we can select which side of the lander
 the robot thinks it is hanging off of at the start of the match and detect the
 gold with TensorFlow.  After "Play" is pressed, Jack grabs the correct element
 with the intake, places the Cat Hat team marker into our depot, and extends the
 arm into either crater to count as parking according to what we chose during
 init using gamepad1 before the start of the match.

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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="State Autonomous", group="CatAuto")
public class MecAutonomousLevel3_State extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHW robot = new CatMecanumHW();  // Use our mecanum hardware
    CatVisionHW eyes = new CatVisionHW();     // Doge and vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = false;
    private boolean parkInOurCrater = true;
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

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status: ", "Initializing: (Resetting Encoders...)");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();


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


        // Lower the robot and begin!
        robot.lowerRobot();
        // Drive the robot out the hook
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,4.5,2.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(.4,0,1);

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
    //recenters to 0 degrees
        robot.mecTurn(.4,2,1);

        switch (samplingPos) {
            case LEFT:
                robot.mecTurn(robot.TURN_SPEED,-12,2.5);
                break;
            case RIGHT:
                robot.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        //Pick up gold
        robot.rotateArm(CatMecanumHW.ARM_FLOOR);
        robot.robotWait(.5);
        robot.extendArm();
        robot.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        robot.retractArm();
        robot.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);



        //drives forward to avoid hiting the landers leg
        robot.mecTurn(CatMecanumHW.TURN_SPEED,-26,3);
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,10,3.0,CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHW.TURN_SPEED ,-84,3.5);
        //drives toward wall and aims toward the craytor
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,27,6,CatMecanumHW.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHW.TURN_SPEED,-110,2);
        //lower arm
        robot.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        //Extend arm to depot
        robot.extendArm();
        //Spit out team marker
        robot.intakeServo.setPower(-0.87);
        robot.robotWait(.5);
        //Bring back arm
        robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.intakeServo.setPower(0.0);
        robot.retractArm();
        //lifts arm to avoid hitting the lander's leg
        robot.rotateArm(CatMecanumHW.ARM_TUCKED_IN);
        //turns to aim at the sample field
        robot.mecTurn(0.9,12,3);
        robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.extendArm();
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
        robot.mecTurn(.4,2,1);
        // Drive ahead to deploy marker
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 10, 2, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        //Lower arm
        robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        //Extend arm to depot
        robot.extendArm();
        //Finish lower arm
        robot.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        //Spit out team marker
        robot.intakeServo.setPower(-0.87);
        robot.robotWait(0.5);
        //Bring back arm
        robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
        robot.intakeServo.setPower(0.0);
        robot.retractArm();
        // Drive back to hit gold
        robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, -10, 2, CatMecanumHW.DRIVE_MODE.driveTilDistance);
        //Turn to gold
        switch (samplingPos) {
            case LEFT:
                robot.mecTurn(robot.TURN_SPEED,-19,2.5);
                break;
            case RIGHT:
                robot.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        //Pick up gold
        robot.rotateArm(CatMecanumHW.ARM_FLOOR);
        robot.intakeServo.setPower(0.87);
        robot.extendArm();
        // Pull the arm back in
        robot.retractArm();
        robot.intakeServo.setPower(0.0);
        //Pick up arm
        robot.rotateArm(CatMecanumHW.ARM_STRAIGHT_UP);
        //sets LEDs to color of the alliance
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }

        if (parkInOurCrater) {
            // Drive to crater nearest the audience
            robot.mecTurn(CatMecanumHW.TURN_SPEED ,27,3.5);
            robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,12,3.0,CatMecanumHW.DRIVE_MODE.driveTilDistance);
            if (closeToWall)
            {
                robot.mecTurn(CatMecanumHW.TURN_SPEED, 73, 3.5);
                //Lower arm
                robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 32, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
                robot.mecTurn(CatMecanumHW.TURN_SPEED, 110, 1);
                robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            }
            else {
                robot.mecTurn(CatMecanumHW.TURN_SPEED, 78, 3.5);
                //Lower arm
                robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
                robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 14, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
                robot.mecTurn(CatMecanumHW.TURN_SPEED, 120, 1);
                robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED, 6, 3.0, CatMecanumHW.DRIVE_MODE.driveTilDistance);
            }
        } else {
            // Drive to farther crater
            robot.mecTurn(CatMecanumHW.TURN_SPEED ,-27,3.5);
            robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,10,3.0,CatMecanumHW.DRIVE_MODE.driveTilDistance);
            robot.mecTurn(CatMecanumHW.TURN_SPEED ,-85,3.5);
            //Lower arm
            robot.rotateArm(CatMecanumHW.ARM_OVER_SAMPLING);
            robot.mecDriveVertical(CatMecanumHW.DRIVE_SPEED,14,3.0,CatMecanumHW.DRIVE_MODE.driveTilDistance);
            robot.mecTurn(CatMecanumHW.TURN_SPEED,-100,1);
            robot.rotateArm(CatMecanumHW.ARM_DEPOT_DROPOFF);
        }

        //Extend arm to crater
        robot.extendArm();
        //tries to pick up minerals
        robot.rotateArm(CatMecanumHW.ARM_FLOOR);
        robot.intakeServo.setPower(0.87);
        robot.robotWait(2.0);
        robot.intakeServo.setPower(0.0);

    }
}
