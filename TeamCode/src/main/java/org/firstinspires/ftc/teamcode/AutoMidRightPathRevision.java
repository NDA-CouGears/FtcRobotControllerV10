package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "MiddleRightPath2", group = "OpModes")
public class AutoMidRightPathRevision extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            // Lock the sample in place
            closeClaw();
            armUp();

            // Lift sample to above second bar height
            liftBarUpNoWait();
            // Drive lose the the submersible
            driveToDistance(0.9, 9, 0);
            waitForLift();
            // Drive until sample is over second bar
            driveToDistance(0.7, 5.25, 0);
            // Lock the sample onto the bar
            liftDown();
            // Back away, turn toward other team colored specimens
            driveToDistance(0.9, 14, 0);
            // Drive close enough to wall to be sure range sensor sees it and drive close to wall
            slide(0.9, -10, 0);
            driveToDistance(0.9, 0, 25, 0, false, false);
            driveStraight(0.9, 30, 0);
            driveToDistance(0.9,0, 15, 0,true, false );
            driveStraight(0.9, -39, 0);
            driveToDistance(0.9, 24, 0,0, true, false);
            //place arm down to grab specimin
            turnToHeading(0.8, -180);
            turnToHeading(0.2, -180);
            driveToDistance(0.9, 27, -180);
            openClaw();
            armDown();
            sleep(2000);
            driveStraight(0.9, 4, 0);
            sleep(200);
            closeClaw();
            sleep(100);
            armUp();
            turnToHeading(0.8, 0);
            turnToHeading(0.2, 0);
            slide(0.9, 40, 0);
            liftBarUpNoWait();
            // Drive lose the the submersible
            driveToDistance(0.9, 8, 0);
            waitForLift();
            // Drive until sample is over second bar
            driveToDistance(0.5, 5.25, 0);
            // Lock the sample onto the bar
            liftDown();


        }
    }

}
