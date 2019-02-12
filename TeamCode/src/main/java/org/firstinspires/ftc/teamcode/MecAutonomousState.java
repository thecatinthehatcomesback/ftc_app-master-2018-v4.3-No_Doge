/**
 MecAutonomousState.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of at the start of the match,
 detect the gold with TensorFlow, hit the correct element, place the  Cat Hat
 team marker in our depot, and park in either crater according to what we chose
 using gamepad1 at the beginning of the match.

 MecStateAutonomous is written for the Minnesota State competition to account for the
 changes to our robot since the December tournament including a new arm and intake.
 (February 8-9, 2019)

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="State Autonomous", group="CatAuto")
public class MecAutonomousState extends LinearOpMode {

    /* Declare OpMode members. */
    CatMecanumHardware robot = new CatMecanumHardware();  // Use our mecanum hardware
    CatVisionHardware eyes = new CatVisionHardware();     // Doge and vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = false;
    private boolean parkInOurCrater = true;
    private boolean closeToWall = true;

    private CatVisionHardware.samplingPos samplingPos = CatVisionHardware.samplingPos.RIGHT;

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


        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
          */
        robot.IMUinit();

        /**
         * Find and store the values of the sampling right away
         * while we are hanging to maximize our camera's PoV
         * (Point of View)
          */
        samplingPos = eyes.findGoldPos();
        // Close down the vision to reduce RAM usage
        eyes.tfod.deactivate();

        // Lower the robot and begin!
        robot.lowerRobot();
        // Drive the robot out the hook
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,4.5,2.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
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
            case UNKNOWN:
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
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
            case UNKNOWN:
                robot.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        //Pick up gold
        robot.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.robotWait(.5);
        robot.extendArm();
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        robot.retractArm();
        robot.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);



        //drives forward to avoid hiting the landers leg
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,-26,3);
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,10,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED ,-84,3.5);
        //drives toward wall and aims toward the craytor
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,27,6,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,-110,2);
        //lower arm
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Extend arm to depot
        robot.extendArm();
        //Spit out team marker
        robot.intakeServo.setPower(-0.87);
        robot.robotWait(.5);
        //Bring back arm
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.intakeServo.setPower(0.0);
        robot.retractArm();
        //lifts arm to avoid hitting the lander's leg
        robot.rotateArm(CatMecanumHardware.ARM_TUCKED_IN);
        //turns to aim at the sample field
        robot.mecTurn(0.9,12,3);
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.extendArm();
        /**robot.intakeServo.setPower(-.87);
        //tries to sample from the side
        switch (samplingPos) {
            case LEFT:
                break;
            case CENTER:
                robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,-7,2.5,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
                robot.extendArm();
                break;
            case RIGHT:
                robot.extendArm();
                robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,5,2.5,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
                break;
            case UNKNOWN:
                break;
        }
        robot.mecTurn(robot.TURN_SPEED, 77, 1.5);
        robot.robotWait(.3);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,90,1.5);
        robot.intakeServo.setPower(0.0);
        //puts the arm into the crater
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        if (samplingPos==CatVisionHardware.samplingPos.LEFT) {
            robot.extendArm();
        }
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,25,2.5);
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        //tries to pick up minerals
        robot.intakeServo.setPower(.87);
        robot.robotWait(.75);
        robot.intakeServo.setPower(0.0);
*/

    }
    public void driveDepot() throws InterruptedException {
        robot.mecTurn(.4,2,1);
        // Drive ahead to deploy marker
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 10, 2, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        //Lower arm
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        //Extend arm to depot
        robot.extendArm();
        //Finish lower arm
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Spit out team marker
        robot.intakeServo.setPower(-0.87);
        robot.robotWait(0.5);
        //Bring back arm
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.intakeServo.setPower(0.0);
        robot.retractArm();
        // Drive back to hit gold
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, -10, 2, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        //Turn to gold
        switch (samplingPos) {
            case LEFT:
                robot.mecTurn(robot.TURN_SPEED,-19,2.5);
                break;
            case RIGHT:
            case UNKNOWN:
                robot.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        //Pick up gold
        robot.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.intakeServo.setPower(0.87);
        robot.extendArm();
        // Pull the arm back in
        robot.retractArm();
        robot.intakeServo.setPower(0.0);
        //Pick up arm
        robot.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);
        //sets LEDs to color of the alliance
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }

        /// TODO: 2/4/2019 Maybe a center versus wall option so that we don't hit alliance partner
        if (parkInOurCrater) {
            // Drive to crater nearest the audience
            robot.mecTurn(CatMecanumHardware.TURN_SPEED ,27,3.5);
            robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,12,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
            if (closeToWall)
            {
                robot.mecTurn(CatMecanumHardware.TURN_SPEED, 73, 3.5);
                //Lower arm
                robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 32, 3.0, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
                robot.mecTurn(CatMecanumHardware.TURN_SPEED, 110, 1);
                robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
            }
            else {
                robot.mecTurn(CatMecanumHardware.TURN_SPEED, 78, 3.5);
                //Lower arm
                robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
                robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 14, 3.0, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
                robot.mecTurn(CatMecanumHardware.TURN_SPEED, 120, 1);
                robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 6, 3.0, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
            }
        } else {
            // Drive to farther crater
            robot.mecTurn(CatMecanumHardware.TURN_SPEED ,-27,3.5);
            robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,10,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
            robot.mecTurn(CatMecanumHardware.TURN_SPEED ,-85,3.5);
            //Lower arm
            robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
            robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,14,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
            robot.mecTurn(CatMecanumHardware.TURN_SPEED,-100,1);
            robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        }

        //Extend arm to crater
        robot.extendArm();
        //tries to pick up minerals
        robot.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.intakeServo.setPower(0.87);
        robot.robotWait(2.0);
        robot.intakeServo.setPower(0.0);

    }
}
