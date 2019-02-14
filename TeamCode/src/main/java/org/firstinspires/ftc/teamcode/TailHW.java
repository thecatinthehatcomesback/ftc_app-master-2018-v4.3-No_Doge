/*
      TailHW.java

        An "hardware" class intended to contain common code for accessing the
        hardware specific to moving the tail of the robot.
        This is a modified (stripped down) version of CatMechanumHardware to
        contain only movement for the tail of the robot.

        This file is used by CatAsyncHardware to run multiple motors at once

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class TailHW
{


    /* Public OpMode members. */
    // Motors

    public DcMotor  tailMotor        = null;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;
    CatAsyncHardware mainHW     = null;

    /* Constructor */
    public TailHW(CatAsyncHardware mainHardware){

        mainHW = mainHardware;
        opMode = mainHW.opMode;
        hwMap = mainHW.hwMap;

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        tailMotor        = hwMap.dcMotor.get("tail_motor");


        // Define motor directions //
        tailMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes //
        tailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to run at no power so that the robot doesn't move during init //
        tailMotor.setPower(0);
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
                //robotWait(.3);
                tailMotor.setPower(0.0);
                break;
            }
        }

        // Make sure we stop tail
        tailMotor.setPower(0.0);
        // Pull tail back into the robot (not there yet...)
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