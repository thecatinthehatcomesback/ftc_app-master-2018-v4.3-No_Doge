/*
        CatSubsystemHW.java

    A "hardware" class containing common code accessing hardware objects
    and processes.  It detects if the subclasses are busy and can/should
    continue to the next step/segment of code.  This file is used by
    CatAsyncHW to run multiple motors at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class CatSubsystemHW
{

    /* local OpMode members. */
    public HardwareMap hwMap        = null;
    public CatAsyncHW mainHW        = null;

    /* Constructor */
    public CatSubsystemHW(CatAsyncHW mainHardware){

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


    public static void waitUntillDone(CatSubsystemHW subOne, CatSubsystemHW subTwo){
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

    public static void waitUntillDone(CatSubsystemHW subOne, CatSubsystemHW subTwo, CatSubsystemHW subThree){
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