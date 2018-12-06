/*
      CatVisionHardware.java

        An "hardware" class intended to contain common code for accessing the hardware
        This is a modified (stripped down) version of CatBotHardware to
        be used with Mecanum drivetrain.  This is mostly to test the machine vision

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all vision coding we use.
 * In this case that robot is the Cat in the Hat robot for 2018-2019
 *
 */
public class CatVisionHardware
{
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    enum samplingPos {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
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