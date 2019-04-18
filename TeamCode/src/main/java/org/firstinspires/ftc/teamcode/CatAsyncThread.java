/*
        CatAsyncThread.java

    An helper class containing code to be used when a thread is needed that
    keeps polling the waitUnillDone() method of a subsystem.

*/

package org.firstinspires.ftc.teamcode;

/**
 * This is NOT an OpMode.
 *
 * This class is used to spawn a wait for a subsystem to be done action
 *
 */
public class CatAsyncThread extends Thread {
    CatSubsystemHW subsystem;

    /* Constructor */
    public CatAsyncThread(CatSubsystemHW newSubsystem){
        subsystem = newSubsystem;
    }


    public void run() {
        subsystem.waitUntillDone();
    }
}// End of class bracket