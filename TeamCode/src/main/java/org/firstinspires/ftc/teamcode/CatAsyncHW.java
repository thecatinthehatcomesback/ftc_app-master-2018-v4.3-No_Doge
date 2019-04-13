/*
        CatAsyncHW.java

    An "hardware" class containing common code accessing hardware specific
    to make our Autonomous routines "asynchronous".  This file is used by
    the new autonomous OpModes to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the arm specific hardware for the robot to
 * allow for multiple operations during autonomous.  In this case, that robot is
 * Jack from the Cat in the Hat Comes Back team during the 2018-2019 season.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatAsyncHW
{
    // Wheel measurements
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // Accurate for a NeveRest Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Autonomous Drive Speeds
    static final double     DRIVE_SPEED             = 0.7;
    static final double     HYPER_SPEED             = 0.95;
    static final double     CHILL_SPEED             = 0.45;
    static final double     CREEP_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.6;


    // Arm positions
    static final int        ARM_FLOOR               = 6500; // was 6680
    static final int        ARM_DEPOT_DROPOFF       = 5700;
    static final int        ARM_OVER_SAMPLING       = 5215;
    static final int        ARM_STRAIGHT_UP         = 2550;
    static final int        ARM_TUCKED_IN           = 1800;
    static final int        ARM_STOWED              = 0;



    /* Public OpMode members. */

    //LED stuff
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public static boolean isRedAlliance = true;

    // Sensors
    public ModernRoboticsI2cRangeSensor landerSeer  = null;
    public ColorSensor frontLeftColor               = null;
    public ColorSensor frontRightColor              = null;
    public ColorSensor rearLeftColor                = null;
    public ColorSensor rearRightColor               = null;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;


    // other Hardware subSystems
    CatArmHW arm        = null;
    CatDriveHW drive    = null;
    CatTailHW tail      = null;
    CatExtendHW extend  = null;

    /* Constructor */
    public CatAsyncHW(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode)  throws InterruptedException  {

        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        arm = new CatArmHW(this);
        arm.init();
        drive = new CatDriveHW(this);
        drive.init();
        extend = new CatExtendHW(this);
        extend.init();
        tail = new CatTailHW(this);
        tail.init();


        // Blinkin LED stuff //
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        // Define and Initialize Sensors //
        landerSeer       = hwMap.get(ModernRoboticsI2cRangeSensor.class, "lander_seer");
        frontLeftColor   = hwMap.get(ColorSensor.class, "front_left_color");
        frontRightColor  = hwMap.get(ColorSensor.class, "front_right_color");
        rearLeftColor    = hwMap.get(ColorSensor.class, "rear_left_color");
        rearRightColor   = hwMap.get(ColorSensor.class, "rear_right_color");
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

    public void spawnWait(CatSubsystemHW subsystem) {
        CatAsyncThread theThread = new CatAsyncThread(subsystem);
        theThread.start();
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