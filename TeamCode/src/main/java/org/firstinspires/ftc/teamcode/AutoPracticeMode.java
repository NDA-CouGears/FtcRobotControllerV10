package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous( name = "Practice", group = "OpModes")
public class AutoPracticeMode extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {

            driveToDistance(0.9, 24, 0,0, true, false);
            //place arm down to grab specimin
            turnToHeading(0.8, -180);
            turnToHeading(0.2, -180);
            driveToDistance(0.9, 27, -180);
            openClaw();
            armDown();
            sleep(2000);
            driveStraight(0.9, 4, 0);
            sleep(100);
            closeClaw();
            sleep(100);
            armUp();
            turnToHeading(0.5, 0);
        }
    }
}
