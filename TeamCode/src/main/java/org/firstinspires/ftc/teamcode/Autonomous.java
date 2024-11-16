package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends RobotParent {

    protected static final int RIGHT_DIRECTION = 1;

    protected static final int LEFT_DIRECTION = -1;

    private void mTurn (int direction, int inches){
        if (direction == RIGHT_DIRECTION){ // slide to right
            encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
        }
        if (direction == LEFT_DIRECTION){ // slide to left
            encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
        }
    }
    private void driveForward (int direction, int inches){
        if (direction == RIGHT_DIRECTION){ // slide to right
            encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
        }
        if (direction == LEFT_DIRECTION){ // slide to left
            encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
        }
    }


    private void turn (int direction, double degrees){
        if (direction == RIGHT_DIRECTION) { // turn right in a certain degrees
            encoderDrive(DRIVE_SPEED, degrees, -degrees, degrees, -degrees, 10.0);
        }
        if (direction == LEFT_DIRECTION) { // turn right in a certain degrees
            encoderDrive(DRIVE_SPEED, -degrees, degrees, -degrees, degrees, 10.0);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();



    }
}
