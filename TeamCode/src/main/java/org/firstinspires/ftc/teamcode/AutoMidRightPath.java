package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "MiddleRightPath", group = "OpModes")
public class AutoMidRightPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            closeClaw();
            armUp();
            driveToDistance(0.4, 9, 0);
            liftBarUp();
            driveToDistance(0.4, 6, 0);
            liftDown();
            driveToDistance(0.4, 12, 0);
            turnToHeading(0.3, -90);
            driveToDistance(0.4, 15, -90);
            openClaw();
            armSetPosition(0.75, 0);
            turnToHeading(0.3, 0);
            armSetPosition(1,1000);
            closeClaw();
            armUp();
            driveStraight(0.4, -16, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armSetPosition(0.75,0);
            turnToHeading(0.3, 0);
            driveStraight(0.4, 14, 0);
            slide(0.3, -10, 0);
            armSetPosition(1,1000);
            closeClaw();
            armUp();
            slide(0.3, 11, 0);
            driveStraight(0.4, -19, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armUp();



        }
    }

}
