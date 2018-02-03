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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/* ------------------------------------------------------------------
 * This Op Mode Performs General Drive Train and Arm Manipulations
 * via manual control.
 * 	1. Drive train control via GamePad 1
 * 	2. Arm manipulation control via GamePad 2
 *
 * Control Modules
 *  1. Motor Controller Module
 *  2. Motor Controller Module
 *
 * Gamepads
 * 	1. Left/Right sticks: Tank drive operation
 * 	   "A" button: Toggle front/rear of robot
 * 	   "B" button: Cancel reverse operation (i.e., set front as front)
 *	2. Left Stick: Extend/retract arm
 *	   Right Stick: Pivot arm
 * ------------------------------------------------------------------
 */
@TeleOp(name = "Teleop.")
public class CyberTeleOp extends CyberAbstractOpMode {
    // Set Additional Boolean Variables
    boolean bDirection;                      // Flags fwd/rev drive train operation (1 = Arm is front; 0 = Collection is front)
    //boolean lStatus;

    double dBucketPosition;
    int iScoopPosition;
    final static double
            BUCKET_LOAD_POSITION = 0.5,
            BUCKET_LEFT_POSITION = 0.25,
            BUCKET_RIGHT_POSITION = 0.75,
            BUCKET_DELTA = 0.015;
    final static int
            SCOOP_LOAD_POS = 0,
            SCOOP_DOWN_POS = 420,
            SCOOP_DEL = 10;
    //------------------------------------------------------------------
    // Constructor Method  (Not Used)
    //------------------------------------------------------------------
    public CyberTeleOp()
    {
    }

    //------------------------------------------------------------------
    // Robot Initialization Method
    //------------------------------------------------------------------
    @Override
    public void init() {
        // Get references to dc motors and set initial motor modes and directions.
        // NOTE: Although super.init (see CyberAbstractOpMode) attempts to set all motor modes to Reset-Encoders, the
        // next set of operations in this init method overrides those mode commands before they
        // are recieved by the motor controller.
        super.init();

        // Set motor modes for desired telop operation.
        // Note: Run without encoders will allow the driver to manipulate motor power without
        //       limiting distance. Encoders can be monitored to prevent over/under travel, but
        //       overshoot may be significant. On the other hand, run to position will allow the
        // 		 driver to manipulate motor power in teleop, but travel range is limited to the
        //		 target position (i.e., max or min allowed target position.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorALength.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorAJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorScoop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Start robot with the arm side of the robot designated as the front, and the collection
        // side designated as the rear.
        bDirection = true;     // Arm is set as the front side of the robot



        dBucketPosition = BUCKET_LOAD_POSITION;
        iScoopPosition = SCOOP_LOAD_POS;

    } // End OpMode Initialization Method


    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------
    @Override
    public void loop()
    {
        // LEFT/RIGHT DRIVE TRAIN TELEOP CONTROL
        // Gamepad 1 controls the drive train motors
        // If gamepad1.dpad_up is pressed, set arm as the front of the robot
        // if gamepad1.dpad_down is pressed, set collection as the front of the robot)
        // Set motor directions accordingly to compensate for selected robot direction

        // Bucket controls *NOTE: This is a REV servo. The setPosition values are
        // only general, the true limits are programmed via the REV Servo Programmer

        super.loop();

        if (gamepad1.x)
        {
            dBucketPosition -= BUCKET_DELTA;        // Tilt bucket to the left
        }
        else
        {
            if (gamepad1.b)
            {
                dBucketPosition += BUCKET_DELTA;    // Tilt bucket to the right
            }
            else
            {
                if (dBucketPosition > BUCKET_LOAD_POSITION) // Return bucket to the load position
                {
                    dBucketPosition -= BUCKET_DELTA;
                }
                if (dBucketPosition < BUCKET_LOAD_POSITION)
                {
                    dBucketPosition += BUCKET_DELTA;
                }
            }
        }

        dBucketPosition = Range.clip(dBucketPosition,BUCKET_LEFT_POSITION,BUCKET_RIGHT_POSITION);
        servoBucket.setPosition(dBucketPosition);


        // Drive orientation status
        if (gamepad1.dpad_up)
        {
            bDirection = true; // Arm is front.
        }

            if (gamepad1.dpad_down) {
                bDirection = false; // Collection is front
            }




/*
        // Hang-on hook controls
        if (gamepad2.b)
        {
            servoDrop.setPosition(1);
        }

        if (gamepad2.y)
        {
            servoDrop.setPosition(.5);
        }

        if (gamepad2.x)
        {
            servoDrop.setPosition(0);
        }
*/

        if (gamepad1.a)
        {
            iScoopPosition += SCOOP_DEL;
        }
        else
        {
            if (gamepad1.y)
            {
                iScoopPosition -= SCOOP_DEL;
            }
        }

        if (iScoopPosition > SCOOP_DOWN_POS) iScoopPosition = SCOOP_DOWN_POS;
        if (iScoopPosition < SCOOP_LOAD_POS) iScoopPosition = SCOOP_LOAD_POS;
        motorScoop.setTargetPosition(iScoopPosition);
        motorScoop.setPower(.4);




        // Calculate drive train throttle and direction given selected robot direction.
        throttleDrive = -gamepad1.left_stick_y;
        directionDrive = gamepad1.left_stick_x;

        if (bDirection) // Arm is front
        {
            powerRight = throttleDrive - directionDrive;
            powerLeft = throttleDrive + directionDrive;
        }
        else  // Collection is front
        {
            powerRight = -throttleDrive - directionDrive;
            powerLeft = -throttleDrive + directionDrive;
        }
        // Lock position values

        // Clip left and right drive motor power so they never exceed -1 thru 1.
        powerRight = Range.clip(powerRight, -1, 1);
        powerLeft = Range.clip(powerLeft, -1, 1);

        // Scale left and right drive motor power for better control at low power
        powerRight = (float) scaleInput(powerRight);
        powerLeft = (float) scaleInput(powerLeft);

        // Set drive motor power
        motorRight.setPower(powerRight);
        motorLeft.setPower(powerLeft);


        // EXTEND ARM
        // Determine desired direction of arm extension. If positive, set position to the Max;
        // otherwise, set position to the Min. Power polarity does not matter in Go-to-Position
        // mode; the absolute value of power is always applied in this mode.
        if (-gamepad2.left_stick_y >= 0f)  // Driver extending arm
        {
            motorALength.setTargetPosition(LIMIT_ARM_EXT_MAX);
        }
        else  // Driver retracting arm
        {   // If the arm joint is not near 90 degrees, do not allow the arm to fully retract
            if (motorAJoint.getCurrentPosition() <= LIMIT_ARM_PIV_MAX_TO_RETRACT)
            {
                motorALength.setTargetPosition(LIMIT_ARM_EXT_MIN);
            }
            else
            {
                motorALength.setTargetPosition(LIMIT_ARM_EXT_MIN_FOR_ROTATE);
            }
        }


        throttleALength = gamepad2.left_stick_y;  // For Run-to-Position, power polarity does not matter

        // Clip and scale the throttle, and then set motor power.
        throttleALength = Range.clip(throttleALength, -1, 1);
        throttleALength = (float) scaleInput(throttleALength);
        motorALength.setPower(throttleALength);


        // ROTATE ARM
        // Determine desired direction of arm rotation. If positive, set position to the Max;
        // otherwise, set position to the Min and reverse polarity of the stick (Power >= 0).
        // By using min/max targets in the Go-to-Position mode, the motor will not overshoot.
        if (-gamepad2.right_stick_y >= 0f)   // If the arm is not extended ~2", do not allow pivot
        {
            if (motorALength.getCurrentPosition() >= LIMIT_ARM_EXT_MIN_FOR_ROTATE)
            {
                motorAJoint.setTargetPosition(LIMIT_ARM_PIV_MAX);
            }
            else
            {
                motorAJoint.setTargetPosition(LIMIT_ARM_PIV_MAX_TO_RETRACT);
            }
        }
        else
        {
            if (!gamepad2.right_bumper)
            {
                motorAJoint.setTargetPosition(LIMIT_ARM_PIV_MIN);
            }
            else
            {
                motorAJoint.setTargetPosition(-2000);  // Override minimum allowed joint angle if dPadDown pressed
            }
        }

        throttleAJoint = gamepad2.right_stick_y;


      //  telemetry.addData("Mode Length ", motorALength.getChannelMode());
       // telemetry.addData("Mode Joint ", motorAJoint.getChannelMode());


        // Clip and scale the throttle, and then set power.
        throttleAJoint = Range.clip(throttleAJoint, -1, 1);
        throttleAJoint = (float) scaleInput(throttleAJoint);
        motorAJoint.setPower(throttleAJoint);

            // Send telemetry data back to driver station.
            // FYI - If 2 telemetry operations share the same first field, only the last telemetry operation will provide data to android
        if (bDirection)
        {
            telemetry.addData("1. ", "Arm is front");
        }
        else
        {
            telemetry.addData("1. ", "Collection is front");
        }




        telemetry.addData("3. ", "L/R DrvPwr: " + String.format("%.2f", powerLeft) + " / " + String.format("%.2f", powerRight));
        telemetry.addData("4. ", "L/R DrvPos: " + motorLeft.getCurrentPosition() + " / " + motorRight.getCurrentPosition());
        telemetry.addData("5. ", "L/J ArmPwr: " + String.format("%.2f", throttleALength) + " / " + String.format("%.2f", throttleAJoint));
        telemetry.addData("6. ", "L/P ArmPos: " + motorALength.getCurrentPosition() + " / " + motorAJoint.getCurrentPosition());
        telemetry.addData("7. ", "Ly/Lx DriveSticks: " + String.format("%.2f", gamepad1.left_stick_y) + " / " + String.format("%.2f", gamepad1.left_stick_x));
        telemetry.addData("8. ", "ScoopPos: SP =" + iScoopPosition + ", Actual = " + motorScoop.getCurrentPosition());
        telemetry.addData("9. PB a = ", gamepad1.a + ", PB y = " + gamepad1.y);

        telemetry.addData("Color Values","");
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Sat", hsvValues[1]);
        telemetry.addData("V", hsvValues[2]);
        telemetry.addData("Current color Blue?", blue);
        telemetry.addData("Current color Red?", red);
        telemetry.addData("Current color White?", white);

    } // End OpMode Loop Method

    @Override
    public void stop ()
    {
            super.stop();
    }

} // End OpMode


