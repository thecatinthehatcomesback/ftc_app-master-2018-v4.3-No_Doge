/**
 MecAutonomousBasic.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of with gamepad1 and
 detect the gold with DogeCV, hit the right element, place our team marker
 in our depot and park in either crater according to what we say using gamepad1
 at the beginning of the match.

 MecBasic is written to use the most basic approach to our autonomous route
 with the help of mechanical sorting intake and a servo to drop our team marker
 off in the depot.  This autonomous is used for our first qualifier this year
 (November 10, 2018).

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
    CatMecanumHardware robot = new CatMecanumHardware();      // Use our mecanum hardware
    CatVisionHardware eyes = new CatVisionHardware();   // Doge and vision init
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = true;
    private boolean isParkNearCrater = true;

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
        // Init our Machine Vision
        eyes.initVision(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();

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
            telemetry.update();

            /**
             * We don't need to "waitForStart()" here since we've been
             * looping all this time and waiting for opMode to be enabled.
             */

        }

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

        // Find and store the values of the sampling
        samplingPos = eyes.findGoldPos();
        eyes.tfod.deactivate();

        // Lower robot here
        robot.lowerRobot();
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,4.5,2.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        //robot.mecDriveHorizontal(robot.DRIVE_SPEED,-4.5,2.0);

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
        //Delay the amount we selected
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

        //drives forward to avoid hiting the landers leg
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,12,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED ,-90,3.5);
        robot.mecDriveHorizontal(CatMecanumHardware.DRIVE_SPEED,-4,1.5);
        //drives toward wall and aims toward the craytor
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,36,6,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,125,2);
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,6,2,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        //lower arm
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Extend arm to depot
        robot.extendArm();
        //Spit out team marker
        robot.intakeServo.setPower(1.0);
        robot.robotWait(0.5);
        //Bring back arm
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.intakeServo.setPower(0.0);
        robot.retractArm();
        //lifts arm to avoid hitting the lander's leg
        robot.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);
        //turns to aim at the sample field
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,90,3);
        robot.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.intakeServo.setPower(-1.0);
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
        robot.intakeServo.setPower(1.0);
        robot.robotWait(.75);
        robot.intakeServo.setPower(0.0);


    }
    public void driveDepot() throws InterruptedException {
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
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED, 10, 2, CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        //Turn to gold
        switch (samplingPos) {
            case LEFT:
                robot.mecTurn(robot.TURN_SPEED,-30,2.5);
                break;
            case RIGHT:
            case UNKNOWN:
                robot.mecTurn(robot.TURN_SPEED,30,2.5);
                break;
        }
        //Pick up gold
        robot.rotateArm(CatMecanumHardware.ARM_FLOOR);
        robot.intakeServo.setPower(0.87);
        robot.extendArm();
        robot.intakeServo.setPower(0.0);
        // Pull the arm back in
        robot.retractArm();
        //Pick up arm
        robot.rotateArm(CatMecanumHardware.ARM_STRAIGHT_UP);
        //sets LEDs to color of the alliance
        if (isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }
        //Drive to side
        robot.mecTurn(robot.TURN_SPEED,0,2.5);
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,10,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED ,85,3.5);
        //Lower arm
        robot.rotateArm(CatMecanumHardware.ARM_OVER_SAMPLING);
        robot.mecDriveVertical(CatMecanumHardware.DRIVE_SPEED,14,3.0,CatMecanumHardware.DRIVE_MODE.driveTilDistance);
        robot.mecTurn(CatMecanumHardware.TURN_SPEED,95,1);
        //moves closer to crater
        robot.rotateArm(CatMecanumHardware.ARM_DEPOT_DROPOFF);
        //Extend arm to crater
        robot.extendArm();
        //tries to pick up minerals
        robot.intakeServo.setPower(0.87);
        robot.robotWait(2.0);
        robot.intakeServo.setPower(0.0);

    }
}
