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

public class AutoMode extends RobotParent {


    /* @Tess. These are all the methods for driving, slide just means to go sideways.
    For doing any forward movement, just init a new encoderDrive and manually put in the
    the inches you want to travel. ( DRIVE_SPEED and timeout are the same :3 )
    */
    private void slideLeft (int direction, int inches){
        encoderDrive(DRIVE_SPEED, -inches , inches, inches, -inches, 10.0);
    }
    private void slideRight (int direction, int inches){
        encoderDrive(DRIVE_SPEED, inches , -inches, -inches, inches, 10.0);
    }
    private void turnLeft (int direction, int inches){
        encoderDrive(DRIVE_SPEED, -inches , inches, inches, -inches, 10.0);
    }
    private void turnRight (int direction, int inches) {
        encoderDrive(DRIVE_SPEED, inches , -inches, -inches, inches, 10.0);
    }

    /* @Tess. These are the 90 turning methods from last year, I will probably have to change
    the encoder drive values later to account for the new wheels, so this is commented out for now.

    protected void turnLeft90(){
        encoderDrive(DRIVE_SPEED,   -17.5, 17.5, -17.5, 17.5,4.0);
    }
    protected void turnRight90(){
        encoderDrive(DRIVE_SPEED,   17.5, -17.5, 17.5, -17.5,4.0);
    }
    */
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();



    }
}
