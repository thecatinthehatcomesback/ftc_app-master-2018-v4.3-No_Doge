/*
      ExtendHW.java

        An "hardware" class intended to contain common code for accessing
        hardware that extends and retracts the arm.
        This is a modified (stripped down) version of CatMecanumHardware to
        be used with only the extension and retraction of the arm.

        This file is used by CatAsyncHardware to run multiple motors at once

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class ExtendHW extends HWSubsystem
{

    static final double     EXTEND_POWER            = -0.95;
    static final double     RETRACT_POWER            = 0.7;

    /* Public OpMode members. */
    // Motors
    public DcMotor  extenderMotor    = null;


    ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public ExtendHW(CatAsyncHardware mainHardware){

        super(mainHardware);
    }



    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        extenderMotor    = hwMap.dcMotor.get("extendey");

        // Define motor directions //
        extenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes //
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to run at no power so that the robot doesn't move during init //
        extenderMotor.setPower(0);

    }


    /**
     * ---   _____________________   ---
     * ---   Arm Movement Patterns   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void extendArm() {
        /**
         * Simply throw the intake on the end
         * of the arm outwards.
         */
        extenderMotor.setPower(EXTEND_POWER);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < 0.7){
            if (!mainHW.opMode.opModeIsActive()) {
                return;
            }
        }
    }
    public void retractArm() {
        /**
         * Simply pull back the intake on the
         * end of the arm outwards.
         */
        extenderMotor.setPower(RETRACT_POWER);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
    }

    @Override
    public boolean isDone() {
        return !(runtime.seconds() < 0.7);
    }



    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */

}// End of class bracket