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

    /*protected static final int RIGHT_DIRECTION = 1;

    protected static final int LEFT_DIRECTION = -1;

    private void mTurn (int direction, int inches){
        if (direction == RIGHT_DIRECTION){ // slide to right
            encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
        }
        if (direction == LEFT_DIRECTION){ // slide to left
            encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
        }
    }
    */
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Declare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToX(10)
                .build();

        /*Action Turn90RightAction = drive.actionBuilder(drive.pose)
                        .mTurn(1, 4);
        */

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1, // Example of a drive action

                        // This action and the following action do the same thing
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Action!");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Only that this action uses a Lambda expression to reduce complexity
                        (telemetryPacket) -> {
                            telemetry.addLine("Action!");
                            telemetry.update();
                            return false; // Returning true causes the action to run again, returning false causes it to cease
                        }

                )
        );


    }
}
