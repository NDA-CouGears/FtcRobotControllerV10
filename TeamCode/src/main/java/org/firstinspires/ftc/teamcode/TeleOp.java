/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Drive")
public class TeleOp extends RobotParent {

    public static double ARM_UP = 0.24;
    public static double ARM_DOWN = 0.615;

    private ElapsedTime runtime = new ElapsedTime();

    private double signPreserveSquare(double value) {

        if (value > 0) {
            return value * value;
        } else {
            return -(value * value);
        }
    }

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();
        runtime.reset();
        String runTime = runtime.toString();

        int lift_hold_pos = armMotor.getCurrentPosition();

        arm.setPosition(ARM_UP);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //mecanum drive
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = signPreserveSquare(gamepad1.left_stick_y * -0.9); // Remember, this is reversed!
            double lateral = signPreserveSquare(gamepad1.left_stick_x * 0.7); // Counteract imperfect strafing
            double yaw = (signPreserveSquare(gamepad1.right_stick_x * 1)) * 0.5;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            int lfp = leftFrontDrive.getCurrentPosition();
            int rfp = rightFrontDrive.getCurrentPosition();
            int lbp = leftBackDrive.getCurrentPosition();
            int rbp = rightBackDrive.getCurrentPosition();

            //arm

            if (gamepad2.a) {
                lift_hold_pos = 2000;
            }
            else if (gamepad2.b) {
                lift_hold_pos = 4700;
            }
            else {
                double armMotorPower = -gamepad2.right_stick_y;

                if (Math.abs(gamepad2.right_stick_y) < 0.01) {
                    armMotor.setTargetPosition(lift_hold_pos);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);

                    while (opModeIsActive() && armMotor.isBusy() && (Math.abs(gamepad1.left_stick_y) < 0.01)) {
                        idle();
                    }
                    armMotor.setPower(0);
                }
                else {
                    if ((touchSensor.isPressed()) && (armMotorPower < 0)) {
                        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        armMotor.setPower(0);
                    }
                    else {
                        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift_hold_pos = armMotor.getCurrentPosition();
                        armMotor.setPower(armMotorPower);
                    }
                }

                telemetry.addData("Motor power: ", armMotor.getPower());
                telemetry.addData("Motor position: ", armMotor.getCurrentPosition());
            }

            //claw
            //pressing right bumper opens claw, left bumper closes claw
            if (gamepad2.right_bumper) {
                claw.setPosition(ClawOpen);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(ClawClosed);
            }

            //second arm (the one that picks up specimens)
            if (gamepad1.dpad_down) {
                arm.setPosition(ARM_DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setPosition(ARM_UP);
            }


            // Show the elapsed game time and wheel power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Encoders lf, rf, lb, rb", "%d, %d, %d, %d", lfp, rfp, lbp, rbp);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}

