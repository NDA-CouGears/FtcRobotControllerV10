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


    private void slideLeft (int direction, int inches){
        encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
    }
    private void slideRight (int direction, int inches){
        encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
    }
    private void turnLeft (int direction, int inches){
        encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
    }
    private void turnRight (int direction, int inches) {
        encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();



    }
}
