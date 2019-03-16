/*
        CatTailHW.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and extension of Jack's tail.  This is a modified/
    stripped  down version of CatMecanumHW to run all of arm extending
    movements.  This file is used by the new autonomous OpModes to run
    multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
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

public class CatTailHW extends CatSubsystemHW
{


    /* Public OpMode members. */
    // Motors

    public DcMotor  tailMotor        = null;

    ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public CatTailHW(CatAsyncHW mainHardware){

    super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        tailMotor       = hwMap.dcMotor.get("tail_motor");


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
        //  Out of hook at Egan competition
        tailPos[1] = 6150;
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



        runtime.reset();
        tailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Get motor down
        extendTail();
        tailMotor.setPower(1.0);

        // Pull tail back into the robot (not there yet...)
    }


    @Override
    public boolean isDone() {
        Log.d("catbot", String.format(" Tail Target %d, current %d, , Time Left %f ",
                tailMotor.getTargetPosition(), tailMotor.getCurrentPosition(), 4.5 - runtime.seconds()));
        if (runtime.seconds() > 4.5) {
            // Turn off the encoders to just back out hard
            tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            tailMotor.setPower(-1.0);
            mainHW.robotWait(0.3);
            tailMotor.setPower(0.0);
            return true;

        }
        return !(tailMotor.isBusy());

    }


    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */

}// End of class bracket