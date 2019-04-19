/*
        CatExtendHW.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and extension of the arm.  This is a modified/stripped
    down version of CatMecanumHW to run all of arm extending movements.
    This file is used by the new autonomous OpModes to run multiple
    operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the specific hardware for the robot to
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
public class CatExtendHW extends CatSubsystemHW
{

    static final double     EXTEND_POWER            = -0.95;
    static final double     RETRACT_POWER           = 0.7;
    static final int        EXTEND_IN               = 0;
    static final int        EXTEND_OUT              = -910;
    static final int        EXTEND_CRATER           = -710;

    boolean                 isEncoder               = false;

    /* Public OpMode members. */
    // Motors
    public DcMotor  extenderMotor    = null;


    ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatExtendHW(CatAsyncHW mainHardware){

        super(mainHardware);
    }



    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        extenderMotor    = hwMap.dcMotor.get("extendey");

        // Define motor directions //
        extenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx extenderMotorEx =(DcMotorEx) extenderMotor;

        extenderMotorEx.setTargetPositionTolerance(10);

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
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isEncoder = false;
        extenderMotor.setPower(EXTEND_POWER);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < 0.6){
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
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isEncoder = false;
        extenderMotor.setPower(RETRACT_POWER);
        runtime.reset();
    }
    public void extendEncoder(int position) {
        extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        isEncoder = true;
        extenderMotor.setTargetPosition(position);
        extenderMotor.setPower(EXTEND_POWER);
        runtime.reset();
    }

    @Override
    public boolean isDone() {
        /**
         * Checking to see when the extends and
         * retracts have finished.
         */
        if (isEncoder) {
            if (!extenderMotor.isBusy()) {
                extenderMotor.setPower(0.0);
                return true;
            }
        }

        if (runtime.seconds() > 0.7) {
            if (extenderMotor.getPower() == RETRACT_POWER) {
                extenderMotor.setPower(0.0);
            }
            Log.d("catbot", String.format(" Arm extend/retract finished TIMEOUT %.2f ", runtime.seconds()));
            return true;
        }
        Log.d("catbot", String.format(" Arm extend/retract waiting %.2f ", runtime.seconds()));
        return false;
    }



    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */

}// End of class bracket