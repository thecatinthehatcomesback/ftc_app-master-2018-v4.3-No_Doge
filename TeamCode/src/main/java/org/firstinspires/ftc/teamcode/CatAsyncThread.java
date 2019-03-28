/*
        CatAsyncHW.java

    An "hardware" class containing common code accessing hardware specific
    to make our Autonomous routines "asynchronous".  This file is used by
    the new autonomous OpModes to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

//TODO FIX THIS!
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
public class CatAsyncThread extends Thread {
    CatSubsystemHW subsystem;

    /* Constructor */
    public CatAsyncThread(CatSubsystemHW newSubsystem){
        subsystem = newSubsystem;
    }


    public void run() {
        /* Placeholder... */
        subsystem.waitUntillDone();
    }
}// End of class bracket