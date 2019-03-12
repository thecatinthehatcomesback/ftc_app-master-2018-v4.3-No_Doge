/*
      HWSubsystem.java

        An "hardware" class intended to contain common code for accessing the hardware
        This contains code that detects if the subclasses are busy or can / should continue
         to the next segment of code.

        This file is used by CatAsyncHardware to run multiple motors at once

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class HWSubsystem
{

    /* local OpMode members. */
    public HardwareMap hwMap           = null;
    public CatAsyncHardware mainHW     = null;

    /* Constructor */
    public HWSubsystem(CatAsyncHardware mainHardware){

        mainHW = mainHardware;
        hwMap = mainHW.hwMap;

    }
    public boolean isDone (){
        return true;
    }

    public boolean isBusy (){
        return !isDone();
    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException {

    }

    public void waitUntillDone(){
        while (!isDone()) {
            if (!(mainHW.opMode.opModeIsActive())){
                return;
            }
        }
    }


    public static void waitUntillDone(HWSubsystem subOne, HWSubsystem subTwo){
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();
        while (subOneBusy || subTwoBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())){
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
        }
    }

    public static void waitUntillDone(HWSubsystem subOne, HWSubsystem subTwo, HWSubsystem subThree){
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();
        boolean subThreeBusy = subThree.isBusy();
        while (subOneBusy || subTwoBusy || subThreeBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())){
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
            subThreeBusy = subThree.isBusy();
        }
    }

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */

}// End of class bracket