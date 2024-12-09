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
            driveToDistance(0.3, 23, 0);
            liftBarUp();
            driveToDistance(0.2, 7, 0);
            liftDown();
            driveToDistance(0.3, -23, 0);
            turnToHeading(0.3, -90);
            driveToDistance(0.3, 45, -90);



        }
    }

}
