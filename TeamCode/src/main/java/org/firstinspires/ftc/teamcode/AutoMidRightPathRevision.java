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
            driveToDistance(0.4, 9, 0);
            waitForLift();
            // Drive until sample is over second bar
            driveToDistance(0.4, 5.5, 0);
            // Lock the sample onto the bar
            liftDown();
            // Back away, turn toward other team colored specimens
            driveToDistance(0.4, 14, 0);
            turnToHeading(0.4, -90);
            // Drive close enough to wall to be sure range sensor sees it and drive close to wall
            driveStraight(0.4, 12, -90);
            driveToDistance(0.4, 25, -90);
            slide(0.4, 32.5, 0);
            driveStraight(0.4, 4, 0);
            driveToDistance(0.4, 16, 0);
            // push samples to observation zone
            slide(0.4, -37, 0);
            slide(0.4, 11, 0);
            turnToHeading(0.2, -180);
            armDown();
            openClaw();
            sleep(2500);
            driveStraight(0.3, 3, 0);
            closeClaw();
            armUp();
            turnToHeading(0.2, 0);
            slide(0.3, 35, 0);
            liftBarUpNoWait();
            // Drive lose the the submersible
            driveToDistance(0.4, 9, 0);
            waitForLift();
            // Drive until sample is over second bar
            driveToDistance(0.4, 6, 0);
            // Lock the sample onto the bar
            liftDown();


        }
    }

}
