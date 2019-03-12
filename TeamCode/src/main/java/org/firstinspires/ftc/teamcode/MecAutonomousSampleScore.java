/**
 MecAutonomousAsync.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of at the start of the match,
 detect the gold with TensorFlow, hit the correct element, place the  Cat Hat
 team marker in our depot, and park in either crater according to what we chose
 using gamepad1 at the beginning of the match.

 MecStateAutonomous is written for the detroit worlds competition to account for the
 changes to our robot since the MN state tournament including an improved arm and intake.
 and runs through the CatAsyncHardware class in order to run many movements at once.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Score Autonomous", group="CatAuto")
public class MecAutonomousSampleScore extends LinearOpMode {

    /* Declare OpMode members. */
    CatAsyncHardware robot = new CatAsyncHardware();  // Use our mecanum hardware
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

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
          */
        robot.drive.IMUinit();
        Log.d("catbot", String.format("IMU initialized - starting lower"));


        // Lower the robot and begin!
        robot.tail.lowerRobot();
        robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.tail.waitUntillDone();
        // Drive the robot out the hook
        robot.drive.mecDriveHorizontal(robot.DRIVE_SPEED,4.5,2.0);
        robot.drive.waitUntillDone();
        robot.drive.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, DriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(.4,2,1);
        robot.drive.waitUntillDone();

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
        robot.arm.rotateArm(CatMecanumHardware.ARM_FLOOR);
        //wait untill both the arm is lowered and the has driven to the correct position to sample
        HWSubsystem.waitUntillDone(robot.arm,robot.drive);
        robot.extend.extendArm();
        robot.extend.waitUntillDone();
        robot.extend.retractArm();
        robot.arm.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);


        //drives forward to avoid hiting the landers leg
        robot.robotWait(.5);
        robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED,-26,3);
        HWSubsystem.waitUntillDone(robot.arm,robot.extend,robot.drive);
        robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,10,3.0,DriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-84,3.5);
        robot.drive.waitUntillDone();
        //drives toward wall and aims toward the craytor
        robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,27,6,DriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED,-110,2);
        robot.drive.waitUntillDone();
        //lower arm
        robot.arm.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Extend arm to depot
        robot.extend.extendArm();
        HWSubsystem.waitUntillDone(robot.arm,robot.extend);
        //Spit out team marker
        robot.arm.intakeServo.setPower(-0.87);
        robot.robotWait(.5);
        //Bring back arm
        robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.arm.waitUntillDone();
        robot.arm.intakeServo.setPower(0.0);
        robot.extend.retractArm();
        robot.extend.waitUntillDone();
        //lifts arm to avoid hitting the lander's leg
        robot.arm.rotateArm(CatMecanumHardware.ARM_TUCKED_IN);
        robot.arm.waitUntillDone();
        //turns to aim at the sample field
        robot.drive.mecTurn(0.9,12,3);
        robot.drive.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.extend.extendArm();
        HWSubsystem.waitUntillDone(robot.arm,robot.extend);
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
        //Extend arm to depot
        robot.extend.extendArm();
        //robot.drive.mecTurn(.4,2,1);
        //robot.drive.waitUntillDone();
        // Drive ahead to deploy marker
        robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 10, 2, DriveHW.DRIVE_MODE.driveTilDistance);
        HWSubsystem.waitUntillDone(robot.arm,robot.drive,robot.extend);
        //drop down tail
        robot.tail.retractTail();
        //Finish lower arm
        robot.arm.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Spit out team marker
        robot.arm.intakeServo.setPower(-0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(0.5);
        //Bring back arm
        robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.arm.intakeServo.setPower(0.0);
        robot.extend.retractArm();
        HWSubsystem.waitUntillDone(robot.extend,robot.arm);
        // Drive back to hit gold
        robot.tail.waitUntillDone();
        robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, -10, 4, DriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHardware.ARM_FLOOR);
        //Turn to gold
        switch (samplingPos) {
            case LEFT:
                robot.drive.mecTurn(robot.TURN_SPEED,-19,2.5);
                break;
            case RIGHT:
                robot.drive.mecTurn(robot.TURN_SPEED,19,2.5);
                break;
        }
        HWSubsystem.waitUntillDone(robot.drive,robot.arm);
        //Pick up gold
        robot.arm.intakeServo.setPower(0.87);
        robot.extend.extendArm();
        robot.extend.waitUntillDone();
        // Pull the arm back in
        //Pick up arm
        robot.arm.rotateArm(CatMecanumHardware.ARM_TUCKED_IN);
        robot.extend.retractArm();
        robot.arm.intakeServo.setPower(0.0);
        robot.drive.mecTurn(.35,0,2);
        robot.arm.setExtenderValue(3000);
        HWSubsystem.waitUntillDone(robot.drive,robot.arm);
        robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
        robot.drive.mecDriveVertical(DriveHW.DRIVE_SPEED,-14,3,DriveHW.DRIVE_MODE.driveTilDistance);

        //robot.extend.extenderMotor.setPower(.7);
        //robot.robotWait(.4);
        //robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
        robot.drive.waitUntillDone();
        robot.arm.gateOpen();
        robot.arm.rotateArm(CatMecanumHardware.ARM_SCORE);
        //robot.arm.rotateArm(CatMecanumHardware.ARM_TUCKED_IN,.8);
        //robot.robotWait(.1);
        //robot.arm.rotateArm(CatMecanumHardware.ARM_SCORE,.65);
        robot.arm.waitUntillDone();
        robot.arm.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);
        robot.arm.waitUntillDone();
        robot.drive.mecDriveVertical(DriveHW.DRIVE_SPEED,3,2,DriveHW.DRIVE_MODE.driveTilDistance);
        robot.drive.waitUntillDone();
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
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,27,3.5);
            HWSubsystem.waitUntillDone(robot.drive,robot.extend);
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,12,3.0,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            if (closeToWall)
            {
                robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED, 73, 3.5);
                robot.drive.waitUntillDone();
                //Lower arm
                robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 32, 3.0, DriveHW.DRIVE_MODE.driveTilDistance);
                robot.drive.waitUntillDone();
                robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED, 110, 1);
                robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
            }
            else {
                robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED, 78, 3.5);
                robot.drive.waitUntillDone();
                //Lower arm
                robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
                robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 14, 3.0, DriveHW.DRIVE_MODE.driveTilDistance);
                robot.drive.waitUntillDone();
                robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED, 120, 1);
                robot.drive.waitUntillDone();
                robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 6, 3.0, DriveHW.DRIVE_MODE.driveTilDistance);
            }
        } else {
            // Drive to farther crater
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-27,3.5);
            HWSubsystem.waitUntillDone(robot.drive,robot.extend);
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,14,3.0,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-85,3.5);
            //Lower arm
            robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
            HWSubsystem.waitUntillDone(robot.drive,robot.arm);
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,14,3.0,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED,-100,1);
            robot.arm.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        }

        //Extend arm to crater
        HWSubsystem.waitUntillDone(robot.drive,robot.arm);
        robot.extend.extendArm();
        robot.robotWait(.4);
        //tries to pick up minerals
        robot.arm.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.arm.intakeServo.setPower(0.87);
        robot.arm.waitUntillDone();
        robot.robotWait(1.0);
        //attempts to score minerals
        if (!parkInOurCrater) {
            robot.arm.rotateArm(CatMecanumHardware.ARM_TUCKED_IN);
            robot.extend.retractArm();
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-90,3.5);
            //robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, -14, 3.0, DriveHW.DRIVE_MODE.driveTilDistance);
            robot.arm.intakeServo.setPower(0.0);
            robot.drive.waitUntillDone();
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-50,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,-18,3.0,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            HWSubsystem.waitUntillDone(robot.extend, robot.arm);
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,6,2);
            robot.robotWait(.4);
            robot.drive.mecTurn(.4,0,2);
            robot.drive.waitUntillDone();
            //scores the minerals
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,-3,2.0,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.extend.extenderMotor.setPower(robot.extend.EXTEND_POWER);
            robot.robotWait(.4);
            robot.arm.gateOpen();
            robot.drive.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHardware.ARM_SCORE,.45);
            //robot.arm.rotateArm(CatMecanumHardware.ARM_TUCKED_IN,.8);
            //robot.robotWait(.1);
            //robot.arm.rotateArm(CatMecanumHardware.ARM_SCORE,.65);
            robot.arm.waitUntillDone();
            robot.arm.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);
            robot.arm.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,30,2,DriveHW.DRIVE_MODE.driveTilDistance);
            robot.drive.waitUntillDone();
            robot.arm.gateClose();
            robot.arm.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
            robot.drive.mecTurn(CatMecanumHardware.TURN_SPEED ,-74,3.5);
            robot.drive.waitUntillDone();
            robot.drive.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 16, 2, DriveHW.DRIVE_MODE.driveTilDistance);
            robot.extend.extendArm();
            robot.drive.waitUntillDone();


        } else {
            robot.arm.intakeServo.setPower(0.0);
        }


    }
}
