/*
      MechanumHardware.java

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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class CatMecanumHardware
{
    // Wheel measurements
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // Accurate for a NeveRest Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Autonomous Drive Speeds
    static final double     DRIVE_SPEED             = 0.45;
    static final double     HYPER_SPEED             = 0.6;
    static final double     CHILL_SPEED             = 0.25;
    static final double     CREEP_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.35;

    // Marker Servo
    static final double     MARKER_IN               = 0.55;
    static final double     MARKER_OUT              = 0.27;

    // Enums!
    enum huh {

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
    public DcMotor  leftBackMotor    = null;
    public DcMotor  rightBackMotor   = null;
    public DcMotor  tailMotor        = null;
    public DcMotor  armMotor         = null;

    //LED stufff
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public static boolean isRedAlliance = true;

    public Servo    identityRelease  = null;
    public Servo    markerServo      = null;

    // Two Vex motors = Continuous Servos
    public CRServo  intakeServo      = null;
    public CRServo  extenderServo    = null;


    // Sensors
    public ModernRoboticsI2cRangeSensor landerSeer  = null;
    public AnalogInput potentiometer                = null;
    public ColorSensor frontLeftColor               = null;
    public ColorSensor frontRightColor              = null;
    public ColorSensor backLeftColor                = null;
    public ColorSensor backRightColor               = null;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;

    /* Constructor */
    public CatMecanumHardware(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode)  throws InterruptedException  {

        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftBackMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightBackMotor   = hwMap.dcMotor.get("right_rear_motor");
        tailMotor        = hwMap.dcMotor.get("tail_motor");
        armMotor         = hwMap.dcMotor.get("arm_motor");

        // blinkin stuff
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);
        // Define and Initialize Servos //
        identityRelease  = hwMap.servo.get("identity_release");
        intakeServo      = hwMap.crservo.get("intakey");
        extenderServo    = hwMap.crservo.get("extendey");
        markerServo      = hwMap.servo.get("markey");
        // Define and Initialize Sensors
        landerSeer       = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lander_seer");
        potentiometer    = hwMap.analogInput.get("potentiometer");
        frontLeftColor   = hwMap.get(ColorSensor.class, "front_left_color");
        frontRightColor  = hwMap.get(ColorSensor.class, "front_right_color");
        backLeftColor    = hwMap.get(ColorSensor.class, "back_left_color");
        backRightColor   = hwMap.get(ColorSensor.class, "back_right_color");

        // Define motor direction //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        tailMotor.setDirection(DcMotor.Direction.FORWARD);
        tailMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set motor modes //
        runNoEncoders();
        tailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set all motors to no power //
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        tailMotor.setPower(0);

        markerServo.setPosition(MARKER_IN);
    }



    /**
     * ---   _____________________   ---
     * ---   Servo control methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void markerRelease() {
        // Set the marker out
        markerServo.setPosition(MARKER_OUT);
    }
    public void markerIn() {
        // Set the marker servo back in
        markerServo.setPosition(MARKER_IN);
    }

    /**
     * ---   _______________________   ---
     * ---   Driving chassis methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        /**
         * Simply setting the powers of each motor in less characters
         */
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);
    }
    public void resetEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runNoEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runToPosition(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void mecDriveVertical(double power, double distance, double timeoutS)  throws InterruptedException {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight vertically
         */

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        ElapsedTime runtime = new ElapsedTime();
        boolean keepDriving = true;

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
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

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

                int leftFrontPosition = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition = leftBackMotor.getCurrentPosition();
                int rightBackPosition = rightBackMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                        !leftBackMotor.isBusy() || !rightBackMotor.isBusy()) {
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
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

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
                int leftBackPosition = leftBackMotor.getCurrentPosition();
                int rightBackPosition = rightBackMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftBackMotor.isBusy() && !rightBackMotor.isBusy()) {
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
// TODO: 12/3/2018 Continue work on this....

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
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

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
                int leftBackPosition   = leftBackMotor.getCurrentPosition();
                int rightBackPosition  = rightBackMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftBackMotor.isBusy() && !rightBackMotor.isBusy()) {
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
        int prevGyro = 0;
        int gyroLoopCount = 0;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            int targetAngleZ;
            targetAngleZ  = -degrees;
            boolean clockwiseTurn = (getCurrentAngle() < degrees);

            // Don't use encoders.  We only use the gyro angle to turn
            runNoEncoders();
            // reset the timeout time and start motion.
            runtime.reset();

            // Change the power based on which angle we are turning to
            if (clockwiseTurn) {
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(-power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(-power);
            } else {
                leftFrontMotor.setPower(-power);
                rightFrontMotor.setPower(power);
                leftBackMotor.setPower(-power);
                rightBackMotor.setPower(power);
            }
            // keep looping while we are still active, and there is time left, and both motors are running.
            int wrapAdjust = 0;
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                int zVal = getCurrentAngle();

                if (prevGyro != zVal) {

                    prevGyro = zVal;
                    gyroLoopCount ++;
                }
                zVal = zVal + wrapAdjust;
                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    break;
                }
                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    break;
                }
                Log.d("catbot", String.format("target %d, current %d", targetAngleZ, zVal));
                opMode.telemetry.addData("Path1",  "Running to %4d", targetAngleZ);
                opMode.telemetry.addData("Path2", "Current angle is %4d" ,zVal);
                opMode.telemetry.update();

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
        tailPos[1] = 8300;
        //  Way high out of hook at our own field
        tailPos[2] = 8500;

    };
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
     * ---   ______________   ---
     * ---   Common Methods   ---
     * ---   \/ \/ \/ \/ \/   ---
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
        tailMotor.setTargetPosition(-8300);
        tailMotor.setPower(1.0);
        // Wait until the robot is completely finished
        while(tailMotor.isBusy()){
            robotWait(1.0);
            if (saftey.seconds() > 4.5) {
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

    // Potentiometor //
    public double getPoteniometorAngle() {

        /**
         * Arm folded in        = 0.135V
         * Arm about vertical   = 1.200V
         * Arm extended back    = 3.250V
         *
         *
         * :::MATH:::
         *
         * If
         * 180 = mx + b
         * x = 1.2
         * and
         * 45 = mx + b
         * x = 0.135
         *
         * rise/run must be
         * m = (1.2-0.135) / (180-45) == 126.76
         *
         *
         * 180 = (126.76)(1.2) + b
         * b = 27.888
         */

        double degrees;
        double m = 126.76;
        double x = potentiometer.getVoltage();
        double b = 27.888;

        degrees = (m*x) + b;

        return degrees;
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

        // Just a simple camparison of the two colors to see which is seen
        if (sensorToUse.red() > sensorToUse.blue()) {
            isRed = true;
        } else {
            isRed = false;
        }


        return isRed;
    }
    public boolean findLine(ColorSensor sensorToUse) {
        /**
         * Tell once it finds a line
         */

        boolean lineFound;
        int sensorAlpha = sensorToUse.alpha();

        // TODO: 12/5/2018 Make sure to get some alpha values and stuff
        if (sensorAlpha > (800)) {
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