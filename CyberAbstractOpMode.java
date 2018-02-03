/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

/* ------------------------------------------------------------------
 * This Op Mode is the super class used for Autonomous Control
 *
 * Control Modules
 *  1. Motor Controller Module
 *  2. Motor Controller Module
 *  3. Servo Controller Module
 *
 * Gamepads
 * 	Not used; however, for troubleshooting prior to competition, Gamepad1's "A" button will allow
 * 	the programmer to advance the sequence 1 step.
 * ------------------------------------------------------------------
 */
public abstract class CyberAbstractOpMode extends OpMode
{
    // Set Servos
    protected Servo
            //servoDrop,
            //servoScoop,
            servoBucket;
    // Set DcMotors
    protected DcMotor
            motorRight, motorLeft,                  // Drive Train Motors Left and Right
            motorALength, motorAJoint,              // Arm Manipulation Motors Extend and Pivot
            motorScoop;

    // Set Sensors
   // protected ColorSensor
    //colorSensor;
    // Set controllers
    //DeviceInterfaceModule cdim;

    // Set Boolean Variables
    protected boolean pulseGamePad1A,                   // Used to detect initial press of "A" button on gamepad 1
            pulseCaseMoveDone,                          // Case move complete pulse
            white,                                      // Variables used to detect current color, used for autonomous control.
            red,
            blue,
            whiteLine;

    // Establish Float Variables
    protected float targetDrDistInch,                   // Targets for motor moves in sequence (engineering units)
            targetDrRotateDeg,
            targetArmDistInch,
            targetArmRotateDeg,
            powerLeft, powerRight,           // TeleOP: Drive train power (or speed in Run-With-Encoders mode)
            throttleDrive, directionDrive,   // TeleOP: Power and direction variables used to calculate Left/Right drive motor power
            throttleALength, throttleAJoint,  // TeleOP: Arm manipulation motor power
            hsvValues[] = {0F,0F,0F};           // Auto: Values used to determine current color detected
    // Establish Double Variables
    protected double targetPower;                         // General motor power variable (%, -1.0 to 1.0)

    // Establish Integer Variables

    protected int seqRobot,                               // Switch operation integer used to identify sequence step.
            targetPosLeft, targetPosRight,      // Drive train motor target variables (encoder counts)
            targetPosArmExt, targetPosArmPiv;   // Arm motor target variables (encoder counts)

    // Establish Integer Constants
    final static int
            ERROR_DRV_POS = 20,                 // Allowed error in encoder counts following drive train position move
            ERROR_ARM_EXT_POS = 20,             // Allowed error in encoder counts following arm extend move
            ERROR_ARM_PIV_POS = 20,             // Allowed error in encoder counts following arm pivot move
            LIMIT_ARM_EXT_MAX = 8150,           // Maximum limit for extension arm (encoder counts)
            LIMIT_ARM_EXT_MIN = 0,              // Minimum limit for extension arm (encoder counts)
            LIMIT_ARM_EXT_MIN_FOR_ROTATE = 1000,// Minimum value of arm Extension before arm rotation allowed (encoder counts)
            LIMIT_ARM_PIV_MAX = 2000,           // Maximum limit for arm rotation (encoder counts)
            LIMIT_ARM_PIV_MIN = 0,              // Minimum limit for arm rotation (encoder counts)
            LIMIT_ARM_PIV_MAX_TO_RETRACT = 20, // Minimum value of Arm rotation before arm can fully retract (encoder counts)
            LED_CHANNEL = 5;





    // Establish Float Constants
    final static float
            ENCODER_CNT_PER_IN_DRIVE = 81.49f,  // (28 count/motor rev x 40 motor rev / shaft rev) / (4 3/8" dia. wheel x pi)
            DEGREE_PER_IN_TRAVEL = 7.5f,        // Amount of rotation for distance travelled (-Left:Right); determined by experiment.
            ENCODER_CNT_PER_IN_ARM_EXT = 40.0f, // NOTICE - This value still needs to be calculated!
            ENCODER_CNT_PER_DEG_ARM_PIV = 30.0f,// NOTICE - This value still needs to be calculated!
            WHITE_VAL_LOW = 0.7f,
            WHITE_SAT_HIGH = 0.35f,
            RED_HUE_LOW = 330f,
            RED_HUE_HIGH = 50f,
            RED_SAT_LOW = .5f,
            RED_VAL_LOW = 0.5f,
            BLUE_HUE_LOW = 200f,
            BLUE_HUE_HIGH = 275f,
            BLUE_SAT_LOW = .1f,
            BLUE_VAL_LOW = .3f;
    // Establish Double Constants
    final static double
            DELAY_DRV_MOV_DONE = 0.1d,          // Hold/wait 0.1s after drive train move complete (seconds)
            DELAY_ARM_MOV_DONE = 0.1d,          // Hold/wait 0.1s after arm move complete (seconds)
            STOP_COLOR_SEARCH = 4.0d;

    // Establish Controller and Device String Constants
    // These names need to match the Robot Controller configuration file device names.
    final static String
            MOTOR_DRIVE_LEFT = "left",
            MOTOR_DRIVE_RIGHT = "right",
            MOTOR_ARM_EXTEND = "length",
            MOTOR_ARM_PIVOT = "joint",
            MOTOR_SCOOP = "motorscoop",
            //SERVO_SCOOP = "scoop",
            //SERVO_DROP = "hook",
            SERVO_BUCKET = "bucket";
            //SENSOR_COLOR = "color";

    //------------------------------------------------------------------
    // Robot Initialization Method
    //------------------------------------------------------------------
    @Override
    public void init()
    {
        // Get references to dc motors and set initial mode and direction
        // It appears all encoders are reset upon robot startup, but just in case, set all motor
        // modes to Reset-Encoders during initialization.
        motorLeft = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT);        // Drive train left motor
        motorLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        motorRight = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT);    // Drive train right motor
        motorRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorALength = hardwareMap.dcMotor.get(MOTOR_ARM_EXTEND);    // Extend arm motor
        motorALength.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorALength.setDirection(DcMotor.Direction.FORWARD);

        motorAJoint = hardwareMap.dcMotor.get(MOTOR_ARM_PIVOT);    // Pivot arm motor
        motorAJoint.setDirection(DcMotor.Direction.REVERSE);
        motorAJoint.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorScoop = hardwareMap.dcMotor.get((MOTOR_SCOOP));
        motorScoop.setDirection(DcMotor.Direction.FORWARD);
        motorScoop.setMode(DcMotor.RunMode.RESET_ENCODERS);

        //servoDrop = hardwareMap.servo.get(SERVO_DROP);
        servoBucket = hardwareMap.servo.get(SERVO_BUCKET);
        //servoScoop = hardwareMap.servo.get(SERVO_SCOOP);
        //Initialize the autonomous sequence to step/case 1
        seqRobot = 1;    // Set seqRobot = 1 to kick off the sequence.

        // get a reference to our DeviceInterfaceModule object.
        //cdim = hardwareMap.deviceInterfaceModule.get("dim");        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        //cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        //colorSensor = hardwareMap.colorSensor.get(SENSOR_COLOR);

        servoBucket.setPosition(0.5);
        //servoScoop.setPosition(.95);
        //servoDrop.setPosition(0);

        //cdim.setDigitalChannelState(LED_CHANNEL,true);

        whiteLine = false;

    } // End OpMode Initialization Method

    //------------------------------------------------------------------
    // Loop Method
    //------------------------------------------------------------------
    @Override
    public void loop()
    {
        /*
        Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);
        if (hsvValues[2]> WHITE_VAL_LOW && hsvValues[1] < WHITE_SAT_HIGH)
        {
            white = true;
        }
        else
        {
            white = false;
        }

        if ( hsvValues[1] > RED_SAT_LOW && hsvValues[2] > RED_VAL_LOW)
        {
            if (hsvValues[0] > RED_HUE_LOW || hsvValues[0] < RED_HUE_HIGH)
            {
                red = true;
            }
            else
            {
                red = false;
            }
        }
        else
        {
            red = false;
        }

        if (hsvValues[0] > BLUE_HUE_LOW && hsvValues[0]< BLUE_HUE_HIGH && hsvValues[1] > BLUE_SAT_LOW && hsvValues[2] > BLUE_VAL_LOW)
        {
            blue = true;
        }
        else
        {
            blue = false;
        }
    */
    }



    //------------------------------------------------------------------
    // Stop Method
    //------------------------------------------------------------------
    @Override
    public void stop()
    {    // stop all the motors when the program is stopped
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorALength.setPower(0);
        motorAJoint.setPower(0);
    } // End OpMode Stop Method


    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    // calcRotate Method
    // Calculate linear distance needed for desired rotation
    // Parameters:
    // 		rotateDeg = Rotation desired (degrees)
    //		anglePerIn = Angle of rotation when left and right move 1 inch in opposite directions
    // Return: linear distance in inches
    float calcRotate(float rotateDeg, float anglePerIn)
    {
        return rotateDeg / anglePerIn;
    }


    // cmdMoveR Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be relative; in other words, motor target is
    // current position plus new distance.
    // Parameters:
    //		distIn = Relative target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveR(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = ((int) (distIn * encoderCntPerIn)) + motor.getCurrentPosition();

        // Set motor target and power
        motor.setPower(power);
        motor.setTargetPosition(target);

        return target;
    }


    // cmdMoveA Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be absolute; in other words, motor target is
    // based on the original home position.
    // Parameters:
    //		distIn = Absolute target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveA(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = (int) (distIn * encoderCntPerIn);

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }


    // chkMove method
    // Verify motor has achieved target
    // Parameters:
    //		motor = motor
    //		target = desired target (encoder counts)
    //		delta = allowed +/- error from target (encoder counts)
    // Return:
    //		True if move complete
    //		False if move not complete
    boolean chkMove(DcMotor motor, int target, int delta)
    {
        int currentPos = motor.getCurrentPosition();
        return ((currentPos >= (target - delta)) && (currentPos <= (target + delta)));
    }

    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    //	scaleInput method
    // (written by unknown FTC engineer)
    // 	This method scales the joystick input so for low joystick values, the
    //	scaled value is less than linear.  This is to make it easier to drive
    //	the robot more precisely at slower speeds.

    double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0)
        {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16)
        {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    // limit method - Recommend not using this method
    // This method prevents over-extended motor movement. Once a limit is reached, you cannot go
    // any further, but you may reverse course. Unfortunately this does not prevent significant
    // overshoot. A better way to limit motor distance is to place the motor into Run-To-Position
    // mode, and then adjust power manually.
    //
    // Method Parameters:
    //     powerValue = desired motor throttle value
    //     direction = (once the encoders are wired properly, this term may go away)
    //     lower limit / upper limit = allowed range of movement
    //
    //  Motor Output:
    //      powerValue = recalculated motor throttle
    //
    //  If time permits, you can revise the code to reduce motor power as a limit is approached.

    float limit(float powerValue, double currentPos, double lowerLimit, double upperLimit)
    {
        if (currentPos > upperLimit)
        {
            if (powerValue > 0)
            {
                powerValue = 0;
            }
        }

        if (currentPos < lowerLimit)
        {
            if (powerValue < 0)
            {
                powerValue = 0;
            }
        }

        return powerValue;
    }

}
