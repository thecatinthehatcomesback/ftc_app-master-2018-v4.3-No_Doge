/*
      ArmHW.java

        An "hardware" class intended to contain common code for accessing the hardware specific to the movement and rotation of the arm
        This is a modified (stripped down) version of CatMechanumHardware to run all of arm movements.

        This file is used by CatAsyncHardware to run multiple motors at once

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
public class ArmHW extends HWSubsystem
{

    /* Public OpMode members. */

    // Gate Servo Constants
    static final double     GATE_OPEN               = 0.80;
    static final double     GATE_CLOSE              = 0.20;

    static final double     ARM_POWER               = 1;

    // Motors
    public DcMotor  armMotor         = null;
    public DcMotorEx armMotorEx      = null;

    // The servo keeping the minerals inside the intake
    public Servo    gateServo        = null;

    // Two Vex motors = Continuous Servos
    public CRServo  intakeServo      = null;

    //creates runtime to tell how long the moters were running
    ElapsedTime runtime = new ElapsedTime();

    /* local OpMode members. */



    /* Constructor */
    public ArmHW(CatAsyncHardware mainHardware){
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {


        // Define and Initialize Motors //
        armMotor         = hwMap.dcMotor.get("arm_motor");



        // Define and Initialize Servos //
        intakeServo      = hwMap.crservo.get("intakey");
        gateServo        = hwMap.servo.get("gate");

        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes //
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotorEx =(DcMotorEx) armMotor;

        armMotorEx.setTargetPositionTolerance(60);
        // Set all motors to run at no power so that the robot doesn't move during init //
        intakeServo.setPower(0);

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

    /**
     * ---   _____________________   ---
     * ---   Arm Movement Patterns   ---
     * ---   \/ \/ \/ \/ \/ \/ \/    ---
     */
    public void rotateArm(int targetPos){
        /**
         * A simple method to move the
         */

        // Set the mode to use encoder
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Start moving to the target position with the correct power
        armMotor.setTargetPosition(targetPos);
        armMotor.setPower(ARM_POWER);

        // Use the timer as a fail-safe in case the

        runtime.reset();

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


    @Override
    public boolean isDone() {
        Log.d("catbot", String.format(" Arm rotate target %d, current %d ", armMotor.getTargetPosition(),armMotor.getCurrentPosition()));

        return !(armMotor.isBusy() && (runtime.seconds() < 3.0));
    }

    @Override
    public boolean pastPosGoingUp(double pos) {
        Log.d("catbot", String.format(" Arm rotate target %d, current %d ", armMotor.getTargetPosition(),armMotor.getCurrentPosition()));

        return !(armMotor.getCurrentPosition() > pos && (runtime.seconds() < 3.0));

    }


    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */

}// End of class bracket