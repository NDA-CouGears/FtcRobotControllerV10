package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous( name = "MiddleRightPath", group = "OpModes")
@Disabled
public class AutoMidRightPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            // Lock the sample in place
            closeClaw();
            armUp();

            // Drive lose the the submersible
            driveToDistance(0.4, 9, 0);
            // Lift sample to above second bar height
            liftBarUpNoWait();
            // Drive until sample is over second bar
            driveToDistance(0.4, 6, 0);
            // Lock the sample onto the bar
            liftDown();
            // Back away, turn toward other team colored specimens
            driveToDistance(0.4, 14, 0);
            turnToHeading(0.3, -90);
            // Drive close enough to wall to be sure range sensor sees it and trive close to wall
            driveStraight(0.4, 15, -90);
            driveToDistance(0.4, 16, -90);
            // Open the claw and lower the arm to grab the first specimen
            openClaw();
            armSetPosition(0.75, 0);
            turnToHeading(0.3, 0);
            armSetPosition(1,1000);
            closeClaw();
            armUp();
            // Back up to base area and drop the specimen
            driveStraight(0.4, -15, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            // Lift the arm and drive back out for the second specimen
            armSetPosition(0.75,0);
            turnToHeading(0.3, 0);
            driveStraight(0.4, 15, 0);
            // Slide over to line up with it then lower arm and grab it.
            slide(0.3, -10, 0);
            armSetPosition(1,1000);
            closeClaw();
            armUp();
            // Slide back so we don't collide with the firt specimen we dropped off
            slide(0.3, 11, 0);
            // Back up and drop the specimen off next to the first one
            driveStraight(0.4, -16, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armUp();



        }
    }

}
