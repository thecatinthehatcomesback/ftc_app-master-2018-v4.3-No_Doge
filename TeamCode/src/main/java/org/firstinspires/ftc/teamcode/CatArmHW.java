/*
        CatArmHW.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the arm.  This is a modified/stripped
    down version of CatMecanumHW to run all of arm movements.  This file
    is used by the new autonomous OpModes to run multiple operations at
    once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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
public class CatArmHW extends CatSubsystemHW
{

    /* Public OpMode members. */

    // Gate Servo Constants
    static final double GATE_OPEN       = 0.80;
    static final double GATE_CLOSE      = 0.20;
    static final double ARM_POWER       = 1.00;
    boolean shouldStartExtend           = false;
    int posStartExtend                  = 3000;

    // Motors
    public DcMotor   armMotor        = null;
    public DcMotorEx armMotorEx      = null;

    // The servo keeping the minerals inside the intake
    public Servo    gateServo        = null;

    // Two Vex motors = Continuous Servos
    public CRServo  intakeServo      = null;

    // Sensors
    public DigitalChannel armRotateLimit = null;
    public DigitalChannel armExtendLimit = null;


    //creates runtime to tell how long the moters were running
    ElapsedTime runtime = new ElapsedTime();

    /* local OpMode members. */



    /* Constructor */
    public CatArmHW(CatAsyncHW mainHardware){
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {


        // Define and Initialize Motors //
        armMotor         = hwMap.dcMotor.get("arm_motor");

        // Define and Initialize Servos //
        intakeServo      = hwMap.crservo.get("intakey");
        gateServo        = hwMap.servo.get("gate");

        // Define and Initialize Sensors //
        armRotateLimit = hwMap.get(DigitalChannel.class,"arm_rotate_limit");
        armExtendLimit = hwMap.get(DigitalChannel.class,"arm_extend_limit");

        // Set motor and servo modes //
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotorEx =(DcMotorEx) armMotor;

        armMotorEx.setTargetPositionTolerance(60);
        // Set all motors to run at no power so that the robot doesn't move during init //
        intakeServo.setPower(0.0);

        // Set Servos to init positions //
        gateServo.setPosition(GATE_CLOSE);
    }


    /**
     * ---   _____________________   ---
     * ---   Servo Control Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void gateOpen() {
        // Set the gate open
        gateServo.setPosition(GATE_OPEN);
    }
    public void gateClose() {
        // Set the gate close
        gateServo.setPosition(GATE_CLOSE);
    }

    boolean isRotateLimit() {
        return (!armRotateLimit.getState());
    }
    /**
     * ---   _____________________   ---
     * ---   Arm Movement Patterns   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void rotateArm(int targetPos){
        /**
         * A simple method to move the
         */

        shouldStartExtend = false;
        // Set the mode to use encoder
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Start moving to the target position with the correct power
        armMotor.setTargetPosition(targetPos);
        armMotor.setPower(ARM_POWER);

        // Use the timer as a fail-safe in case the

        runtime.reset();

    }

    public void setExtenderValue(int newValue){
        posStartExtend = newValue;
        shouldStartExtend = true;
    }




    public void rotateArm(int targetPos, double armPowerIn){
        /**
         * A simple method to move the
         */

        // Set the mode to use encoder
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Start moving to the target position with the correct power
        armMotor.setTargetPosition(targetPos);
        armMotor.setPower(armPowerIn);

        // Use the timer as a fail-safe in case the

        runtime.reset();

    }

    public void hungryHungryHippo() {
        /**
         * During autonomous, we use this to turn on the
         * intake, extend
         */
    }


    static  double TIMEOUT = 3.0;
    @Override
    public boolean isDone() {

        Log.d("catbot", String.format(" Arm rotate target %d, current %d ", armMotor.getTargetPosition(),armMotor.getCurrentPosition()));
        if (shouldStartExtend&&pastPosGoingUp(posStartExtend))
        {
         mainHW.extend.extendArm();
         shouldStartExtend = false;
        }
        if (runtime.seconds() > TIMEOUT) {
            Log.d("catbot", String.format(" Arm rotate TIMEOUT %.2f ", runtime.seconds()));
        }

        return !(armMotor.isBusy() && (runtime.seconds() < TIMEOUT));
    }

    public boolean pastPosGoingUp(int pos) {
        Log.d("catbot", String.format(" Arm rotate target %d, current %d ", armMotor.getTargetPosition(),armMotor.getCurrentPosition()));

        return !(armMotor.getCurrentPosition() > pos);
    }

    public void autoResetArm() {
        /**
         * A algorithm that resets the intake arm rotational
         * encoders during/before autonomous.
         */
        boolean hasReset = false;


        if (!isRotateLimit()) {
            armMotor.setPower(-0.70);
        }

        while (mainHW.opMode.opModeIsActive()) {

            if (!hasReset) {
                if (isRotateLimit()) {
                    armMotor.setPower(0.40);
                    hasReset = true;
                }
            } else {
                if (!isRotateLimit()) {
                    armMotor.setPower(0.00);
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mainHW.robotWait(0.5);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setTargetPosition(-725);
                    armMotor.setPower(0.50);
                    while (armMotor.isBusy()) {
                    }
                    armMotor.setPower(0.00);
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                }
            }
        }
    }
}// End of class bracket