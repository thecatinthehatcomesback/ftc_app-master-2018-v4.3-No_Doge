/*
      CatMechanumHardware.java

        An "hardware" class intended to contain common code for accessing the hardware
        This is a modified (stripped down) version of CatBotHardware to
        be used with mecanum drive train.

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the Cat in the Hat robot for 2018-2019
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have underscores between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatMecanumHW
{
    // Wheel measurements
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // Accurate for a NeveRest Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Autonomous Drive Speeds
    static final double     DRIVE_SPEED             = 0.6;
    static final double     HYPER_SPEED             = 0.8;
    static final double     CHILL_SPEED             = 0.25;
    static final double     CREEP_SPEED             = 0.10;
    static final double     TURN_SPEED              = 0.6;
    static final double     ARM_POWER               = 0.9;


    // Gate Servo Constants
    static final double     GATE_OPEN               = 0.80;
    static final double     GATE_CLOSE              = 0.20;

    // Arm positions
    static final int        ARM_FLOOR               = 6500; // was 6680
    static final int        ARM_DEPOT_DROPOFF       = 5700;
    static final int        ARM_OVER_SAMPLING       = 5215;
    static final int        ARM_EXTEND              = 3750;
    static final int        ARM_STRAIGHT_UP         = 2550;
    static final int        ARM_SLOW                = 1900;
    static final int        ARM_TUCKED_IN           = 1800;
    static final int        ARM_TELEOP              = 1650;
    static final int        ARM_SCORE               = 1250;
    static final int        ARM_STOWED              = 0;
    static final double     EXTEND_POWER            = -0.7;


    // Enums!
    enum DRIVE_MODE {
        findLine,
        followWall,
        driveTilDistance,
        driveUsingGyroStraight
    }


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    /* Public OpMode members. */
    // Motors
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  rightFrontMotor  = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor  tailMotor        = null;
    public DcMotor  armMotor         = null;
    public DcMotor  extenderMotor    = null;

    //LED stuff
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public static boolean isRedAlliance = true;

    // The servo keeping the minerals inside the intake
    public Servo    gateServo        = null;

    // Two Vex motors = Continuous Servos
    public CRServo  intakeServo      = null;
    //public CRServo  extenderServo    = null;


    // Sensors
    public ModernRoboticsI2cRangeSensor landerSeer  = null;
    public ColorSensor frontLeftColor               = null;
    public ColorSensor frontRightColor              = null;
    public ColorSensor rearLeftColor                = null;
    public ColorSensor rearRightColor               = null;
    public DigitalChannel armLimit              = null;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;

    /* Constructor */
    public CatMecanumHW(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode)  throws InterruptedException  {

        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");
        tailMotor        = hwMap.dcMotor.get("tail_motor");
        armMotor         = hwMap.dcMotor.get("arm_motor");
        extenderMotor    = hwMap.dcMotor.get("extendey");

        // Blinkin LED stuff //
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        // Define and Initialize Servos //
        intakeServo      = hwMap.crservo.get("intakey");
        gateServo        = hwMap.servo.get("gate");

        // Define and Initialize Sensors //
        landerSeer       = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lander_seer");
        frontLeftColor   = hwMap.get(ColorSensor.class, "front_left_color");
        frontRightColor  = hwMap.get(ColorSensor.class, "front_right_color");
        rearLeftColor    = hwMap.get(ColorSensor.class, "rear_left_color");
        rearRightColor   = hwMap.get(ColorSensor.class, "rear_right_color");
        armLimit         = hwMap.get(DigitalChannel.class,"arm_limit");

        // Define motor directions //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        tailMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        extenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes //
        runNoEncoders();
        tailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        tailMotor.setPower(0);
        intakeServo.setPower(0);
        extenderMotor.setPower(0);

        // Set Servos to init positions //
        gateServo.setPosition(GATE_CLOSE);
    }



    /**
     * ---   _____________________   ---
     * ---   Servo Control Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void gateOpen() {
        // Set the gate open
        gateServo.setPosition(GATE_OPEN);
    }
    public void gateClose() {
        // Set the gate close
        gateServo.setPosition(GATE_CLOSE);
    }

    /**
     * ---   _____________________   ---
     * ---   Arm Movement Patterns   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void rotateArm(int targetPos){
        /**
         * A simple method to move the
         */

        // Set the mode to use encoder
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Start moving to the target position with the correct power
        armMotor.setTargetPosition(targetPos);
        armMotor.setPower(ARM_POWER);

        // Use the timer as a fail-safe in case the
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(armMotor.isBusy() && (runtime.seconds() < 3.0) ){
            if (!opMode.opModeIsActive()) {
                return;
            }
        }
    }
    public void extendArm() {
        /**
         * Simply throw the intake on the end
         * of the arm outwards.
         */
        extenderMotor.setPower(EXTEND_POWER);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < 0.7){
            if (!opMode.opModeIsActive()) {
                return;
            }
        }
    }
    public void retractArm() {
        /**
         * Simply pull back the intake on the
         * end of the arm outwards.
         */
        extenderMotor.setPower(-EXTEND_POWER);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < 0.7){
            if (!opMode.opModeIsActive()) {
                return;
            }
        }
    }
    public void hungryHungryHippo() {
        /**
         * During autonomous, we use this to turn on the
         * intake, extend
         */
    }

    /**
     * ---   _______________________   ---
     * ---   Driving chassis methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    // Basic all four drive motor power and setMode methods
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        /**
         * Simply setting the powers of each motor in less characters
         */
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);
    }
    public void resetEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runNoEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runToPosition(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Driving Methods:
    public void mecDriveVertical(double power,
                                 double distance,
                                 double timeoutS, DRIVE_MODE driveMode)  throws InterruptedException {
        mecDriveVertical(power, distance, timeoutS, driveMode, null, null);
    }
        public void mecDriveVertical(double power,
                                     double distance,
                                     double timeoutS, DRIVE_MODE driveMode, ColorSensor leftColSen, ColorSensor rightColSen)  throws InterruptedException {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight vertically or using the color sensors to find a
         * line.
         */

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        ElapsedTime runtime = new ElapsedTime();
        boolean keepDriving = true;
        int baseDelta = 0;
        if (driveMode == DRIVE_MODE.findLine) {
            // Turn on the color sensors we want and find the base alpha
            baseDelta = findBaseDelta(rightColSen);
        }

        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget  = (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget  = (int) (distance * COUNTS_PER_INCH);

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runtime.reset();

            // Negate the power if we are going backwards
            if (distance < 0) {
                power = -power;
            }

            drive(power, power, power, power);

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    keepDriving) {

                // Get the current Pos for telemetry only
                int leftFrontPosition  = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition   = leftRearMotor.getCurrentPosition();
                int rightBackPosition  = rightRearMotor.getCurrentPosition();

                // One drive mode that drives blindly straight
                if (driveMode == DRIVE_MODE.driveTilDistance) {
                    //  Exit the method once robot stops
                    if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                            !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                        keepDriving = false;
                    }
                }

                // The other drive mode using color sensors to fine lines
                if (driveMode == DRIVE_MODE.findLine) {
                    // Once left side hits color, turn left side motors off
                    if (findLine(baseDelta, leftColSen)) {
                        leftFrontMotor.setPower(0.0);
                        leftRearMotor.setPower(0.0);

                        if (rightFrontMotor.getPower() == 0.0) {
                            keepDriving = false;
                        }
                        opMode.telemetry.addData("Stop:", "left");
                    }
                    // Once right side hits color, turn right side motors off
                    if (findLine(baseDelta, rightColSen)) {
                        rightFrontMotor.setPower(0.0);
                        rightRearMotor.setPower(0.0);

                        if (leftFrontMotor.getPower() == 0.0) {
                            keepDriving = false;
                        }
                        opMode.telemetry.addData("Stop:", "right");
                    }
                }

                // Display it all for the driver
                opMode.telemetry.addData("New Path",  "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",  "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runtime.seconds());
                opMode.telemetry.update();
            }


            // Stop all motion at the end
            drive(0.0, 0.0, 0.0, 0.0);
        }
    }
    public void mecDriveHorizontal(double power, double distance, double timeoutS) {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight horizontally (positive numbers should strafe left)
         */

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        ElapsedTime runtime = new ElapsedTime();
        boolean keepDriving = true;

        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // (Multiply by sqrt of 2 to compensate)
            newLeftFrontTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightBackTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runtime.reset();

            // Negate the power if we are going right
            if (distance < 0) {
                power = -power;
            }

            // Due to weight diffence, we compensate by cutting the back wheels half power
            drive(power, power, power*0.8, power*0.8);

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    keepDriving) {

                int leftFrontPosition = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition = leftRearMotor.getCurrentPosition();
                int rightBackPosition = rightRearMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                        !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                    keepDriving = false;
                }

                // Log Messages
                /*Log.d("catbot", String.format("encoderDrive targ[%5d,%5d], curr[%5d,%5d] power [%.3f,%.3f]",
                        newLeftTarget,  newRightTarget, leftPosition, rightPosition, leftSpeed, rightSpeed));*/

                // Display it for the driver
                opMode.telemetry.addData("New Path",  "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",  "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runtime.seconds());
                opMode.telemetry.update();
            }


            // Stop all motion
            drive(0, 0, 0, 0);
        }
    }
    public void advMecDrive(double power, double vectorDistance,
                            double vectorAng, double timeoutS) throws InterruptedException {
        /**
         * In this mecanum drive method, we are trying to have the robot
         * drive at an angle while the face of the robot remains pointed
         * ahead.
         *
         *
         * / = back left and forward right motors
         * \ = back right and forward front motors
         *
         * We add the Sin of our angle to the Cos in order to get the
         * powers for \ side of motors while we subtract Sin from Cos
         * for the / side of motors.
         */
// TODO: 1/20/2018 Continue work on this....

        double leftFrontMod  = Math.sin(Math.toRadians(vectorAng))  + Math.cos(Math.toRadians(vectorAng));
        double rightFrontMod = -Math.sin(Math.toRadians(vectorAng)) + Math.cos(Math.toRadians(vectorAng));
        double leftBackMod   = -Math.sin(Math.toRadians(vectorAng)) + Math.cos(Math.toRadians(vectorAng));
        double rightBackMod  = Math.sin(Math.toRadians(vectorAng))  + Math.cos(Math.toRadians(vectorAng));


        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        ElapsedTime runtime = new ElapsedTime();
        boolean keepDriving = true;

        if (opMode.opModeIsActive()) {

            // Determine new target position and multiply each one to adjust for variation of mec wheels
            newLeftFrontTarget  = (int) (vectorDistance * COUNTS_PER_INCH * leftFrontMod);
            newRightFrontTarget = (int) (vectorDistance * COUNTS_PER_INCH * rightFrontMod);
            newLeftBackTarget   = (int) (vectorDistance * COUNTS_PER_INCH * leftBackMod);
            newRightBackTarget  = (int) (vectorDistance * COUNTS_PER_INCH * rightBackMod);

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            // TODO: 11/19/2018 DO WE WANT TO USE RUN TO POSITION????
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runtime.reset();

            // Negate the power if we are going backwards
            if (vectorDistance < 0) {
                power = -power;
            }

            // Calculate motor drive powers after we decide direction
            double SF = findScalor(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);
            leftFrontMod  = leftFrontMod  * SF * power;
            rightFrontMod = rightFrontMod * SF * power;
            leftBackMod   = leftBackMod   * SF * power;
            rightBackMod  = rightBackMod  * SF * power;
            // Drive
            drive(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    keepDriving) {

                // Find the current positions so that we can display it later
                int leftFrontPosition  = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition   = leftRearMotor.getCurrentPosition();
                int rightBackPosition  = rightRearMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftRearMotor.isBusy() && !rightRearMotor.isBusy()) {
                    keepDriving = false;
                }

                // Display it for the driver
                opMode.telemetry.addData("New Path",  "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",  "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runtime.seconds());
                opMode.telemetry.update();
            }


            // Stop all motion
            drive(0, 0, 0, 0);
        }
    }
    public void mecTurn(double power, int degrees, double timeoutS) throws InterruptedException {  //// TODO: 9/20/2018 Look through this and streamline code
        /**
         * Turns counterclockwise with a positive Z angle
         * Turns clockwise with a negative Z angle
         */

        ElapsedTime runtime = new ElapsedTime();
        int gyroLoopCount = 0;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            int targetAngleZ;
            targetAngleZ  = -degrees;
            boolean clockwiseTurn = (getCurrentAngle() > targetAngleZ);

            // Don't use encoders.  We only use the gyro angle to turn
            runNoEncoders();
            // reset the timeout time and start motion.
            runtime.reset();

            // Change the power based on which angle we are turning to
            if (clockwiseTurn) {
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(-power);
                leftRearMotor.setPower(power);
                rightRearMotor.setPower(-power);
            } else {
                leftFrontMotor.setPower(-power);
                rightFrontMotor.setPower(power);
                leftRearMotor.setPower(-power);
                rightRearMotor.setPower(power);
            }
            // keep looping while we are still active, and there is time left, and both motors are running.
            int wrapAdjust = 0;
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                int zVal = getCurrentAngle();

                Log.d("catbot", String.format("target %d, current %d  %s", targetAngleZ, zVal, clockwiseTurn ? "CW": "CCW"));
                opMode.telemetry.addData("Path1",  "Running to %4d", targetAngleZ);
                opMode.telemetry.addData("Path2", "Current angle is %4d" ,zVal);
                opMode.telemetry.update();

                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    break;
                }
                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    break;
                }
                // Allow time for other processes to run.
                opMode.idle();
            }


            // GYRO Telemetry
            if(runtime.seconds() > 0) {
                opMode.telemetry.addData("sample1", "Hz = %.1f", gyroLoopCount / runtime.seconds());
                //DbgLog.msg("GYRO Hz = %.1f  Turn Rate: %.1f", gyroLoopCount / runtime.seconds(), degrees / runtime.seconds());
            }
            opMode.telemetry.update();

            // Stop all motion;
            drive(0, 0, 0, 0);
        }
    }


    /**
     * ---   ____________   ---
     * ---   Tail Methods   ---
     * ---   \/ \/ \/ \/    ---
     */
    public int[] tailPos = new int[3]; {
        /**
         * Each of the encoder ticks that we need to reach the desired heights
         */

        //  All the way down
        tailPos[0] = 0;
        //  Out of hook at Eagan competition
        tailPos[1] = 8250;
        //  Way high out of hook at our own field
        tailPos[2] = 8500;

    }
    public void retractTail() {
        /**
         * Pull tail inside the robot/reattach to the lander
         */
        tailMotor.setTargetPosition(tailPos[0]);
    }
    public void extendTail() {
        /**
         * Extend tail and land the robot
         */
        tailMotor.setTargetPosition(tailPos[1]);
    }

    /**
     * ---   ____________________________   ---
     * ---   Common Miscellaneous Methods   ---
     * ---  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void robotWait(double seconds) {
        ElapsedTime delaytimer = new ElapsedTime();
        while (opMode.opModeIsActive()  &&  (delaytimer.seconds() < seconds)) {
            opMode.idle();
        }
    }
    public double limitRange(double number, double min, double max) {
        return Math.min(Math.max(number, min), max);
    }
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /*
        * Will scale down our power numbers if they are
        * greater than 1.0 so that we continue the set
        * course and don't just limit to the highest
        * possible value...
        */

        /****
         * Look at all motor values
         * Find the highest absolute value (the "scalor")
         * If the highest value is not more than 1.0, we don't need to change the values
         * But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
         * Finally, we pass out the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value
        for(int i = 0; i+1 < values.length; i++){
            if(values[i] > scalor){
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done here...
        if (scalor > 1.0) {
            // Get the reciprocal
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don'thave to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }
    public void lowerRobot() {
        /**
         * Lower our robot from the lander.
         *
         * Slowly expand the tail until completely extended
         * Release the hook
         * Pull tail in all the way
         */

        ElapsedTime saftey = new ElapsedTime();
        saftey.reset();


        tailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Get motor down
        extendTail();
        tailMotor.setPower(1.0);
        // Wait until the robot is completely finished
        while(tailMotor.isBusy()){
            if (saftey.seconds() > 4.5) {
                // Turn off the encoders to just back out hard
                tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                tailMotor.setPower(-1.0);
                robotWait(.3);
                tailMotor.setPower(0.0);
                break;
            }
        }

        // Make sure we stop tail
        tailMotor.setPower(0.0);
        // Pull tail back into the robot (not there yet...)
    }

    /**
     * ---   ___________   ---
     * ---   IMU Methods   ---
     * ---   \/ \/ \/ \/   ---
     */
    public void IMUinit () {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
    }
    public int getCurrentAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }

    /**
     * ---   ____________________   ---
     * ---   Color Sensor Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/   ---
     */
    public boolean isColorRed(ColorSensor sensorToUse) {
        /**
         * Compare red and blue to decide which is seen
         */

        boolean isRed;

        // Just a simple caparison of the two colors to see which is seen
        if (sensorToUse.red() > sensorToUse.blue()) {
            isRed = true;
        } else {
            isRed = false;
        }


        return isRed;
    }
    public int findBaseDelta(ColorSensor colorSensor) {
        /**
         * Before starting to look for a line, find the the current Alpha
         * to add to the threshold so as to give wiggle-room to finding the
         * line.
        */
        int baseDelta = Math.abs(colorSensor.red() - colorSensor.blue());

        return baseDelta;
    }
    public boolean findLine(int baseDelta, ColorSensor colorSensor) {
        /**
         * Tell the robot once a color sensor
         * finds a line.
         */
        boolean lineFound;
        // Take the absolute value of the difference of red and blue
        int currentDelta = Math.abs(colorSensor.red() - colorSensor.blue());

        // Check to see if the line is found
        if (currentDelta > (baseDelta + 70)) {
            lineFound = true;
        } else {
            lineFound = false;
        }

        return lineFound;
    }

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
    public void stuffishable() {
        /* Placeholder... */
    }
}// End of class bracket