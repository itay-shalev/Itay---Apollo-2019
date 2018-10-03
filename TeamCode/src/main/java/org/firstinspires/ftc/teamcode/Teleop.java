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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="mecanum test", group="Pushbot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    //Hardware robot = new Hardware();   // Use a Pushbot's hardwareo

    public double mulltiplieyer = 0.5;
    double leftx;
    double lefty;
    double rightx;
    double righty;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Version 1");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftx = gamepad1.left_stick_x;
            lefty = gamepad1.left_stick_y;
            rightx = gamepad1.right_stick_x;
            righty = gamepad1.right_stick_y;

            if (Math.abs(lefty) >= 0.1 && Math.abs(leftx) <= 0.1 && Math.abs(rightx) <=0.1 && Math.abs(righty) <= 0.1){
                //robot.MotorDriveMode(lefty*mulltiplayer, Hardware.DriveModes.NORMAL);
                telemetry.addData("Say", "strait");
            }
            else if (Math.abs(leftx) >= 0.1 && Math.abs(lefty) <= 0.1 && Math.abs(rightx) <= 0.1 && Math.abs(righty) <= 0.1){
                //robot.MotorDriveMode(leftx*mulltiplayer, Hardware.DriveModes.STRAFE);
                telemetry.addData("Say", "Strafe");
            }
            else if (leftx > 0 && lefty < 0 || leftx < 0 && lefty > 0){
                //robot.MotorDriveMode(leftx, Hardware.DriveModes.DIAGONAL_LEFT);
                telemetry.addData("Say", "Digonal_Left");
            }
            else if (Math.abs(leftx) > 0 && Math.abs(lefty) > 0 || Math.abs(leftx) < 0 && Math.abs(lefty) < 0) {
                //robot.MotorDriveMode(lefty, Hardware.DriveModes.DIAGONAL_RIGHT);
                telemetry.addData("Say", "Diagonal_Right");
            }

            if (Math.abs(rightx) > 0){
                //robot.MotorDriveMode(rightx, Hardware.DriveModes.TURN);
                telemetry.addData("Say", "Turn");
            }
            telemetry.update();
        }
    }
}
