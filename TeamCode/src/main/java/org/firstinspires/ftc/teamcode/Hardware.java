/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  MotorLeftFront   = null;
    public DcMotor  MotorLeftBack  = null;
    public DcMotor  MotorRightFront   = null;
    public DcMotor  MotorRightBack  = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    public enum DriveModes{
        //STRAFE_LEFT,
        STRAFE_RIGHT,
        DIAGONAL_RIGHT_FRONT,
        //DIAGONAL_RIGHT_BACK,
        DIAGONAL_LEFT_FRONT,
        //DIAGONAL_LEFT_BACK,
        TURN,
        NORMAL
    }

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        MotorLeftFront  = hwMap.get(DcMotor.class, "MLF");
        MotorLeftBack = hwMap.get(DcMotor.class, "MLB");
        MotorRightFront  = hwMap.get(DcMotor.class, "MRF");
        MotorRightBack = hwMap.get(DcMotor.class, "MRB");

        MotorLeftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        MotorLeftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        MotorRightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        MotorRightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        SetALLMotorPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        MotorSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void SetALLMotorPower (double Power){
        MotorRightBack.setPower(Power);
        MotorRightFront.setPower(Power);
        MotorLeftBack.setPower(Power);
        MotorLeftFront.setPower(Power);
    }
    public void setLeftMotorPower (double Power){
        MotorLeftBack.setPower(Power);
        MotorLeftFront.setPower(Power);
    }
    public void setRightMotorPower (double Power){
        MotorRightBack.setPower(Power);
        MotorRightFront.setPower(Power);
    }
    public void MotorDriveMode (double Power, DriveModes driveModes) {
        switch (driveModes) {

            case TURN:
                setLeftMotorPower(Power);
                setRightMotorPower(-Power);
                break;
            case DIAGONAL_LEFT_FRONT:
                MotorRightFront.setPower(Power);
                MotorLeftBack.setPower(Power);
                break;
            case DIAGONAL_RIGHT_FRONT:
                MotorLeftFront.setPower(Power);
                MotorRightBack.setPower(Power);
                break;
            case STRAFE_RIGHT:
                MotorLeftBack.setPower(-Power);
                MotorLeftFront.setPower(Power);
                MotorRightBack.setPower(Power);
                MotorRightFront.setPower(-Power);
                break;
    /*        case STRAFE_LEFT:
                MotorLeftBack.setPower(Power);
                MotorLeftFront.setPower(-Power);
                MotorRightBack.setPower(-Power);
                MotorRightFront.setPower(Power);
                break;
    */
            case NORMAL:
                SetALLMotorPower(Power);
                break;
            }
        }


    public void MotorSetMode (DcMotor.RunMode runMode){
        MotorRightFront.setMode(runMode);
        MotorRightBack.setMode(runMode);
        MotorLeftFront.setMode(runMode);
        MotorLeftBack.setMode(runMode);
    }
}

