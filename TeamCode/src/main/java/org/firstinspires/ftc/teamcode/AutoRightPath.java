package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous( name = "RightPath", group = "OpModes")
public class AutoRightPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if(opModeIsActive()) {
            armUp();
            openClaw();
            armSetPosition(0.75, 0);
            driveStraight(0.3, 19, 0);
            slide(0.3, -20, 0);
            turnToHeading(0.2, 0);
            armSetPosition(1,500);
            closeClaw();
            armUp();
            driveStraight(0.3, -16, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armUp();
            armSetPosition(0.75,0);
            turnToHeading(0.3, 0);
            driveStraight(0.3, 16, 0);
            slide(0.3, -10, 0);
            armSetPosition(1,500);
            closeClaw();
            armUp();
            slide(0.3, 11, 0);
            driveStraight(0.3, -19, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armUp();

        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }


    }
}
