package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "MiddlePath", group = "OpModes")
public class AutoMiddlePath extends AutoMode {
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            closeClaw();
            armUp();
            driveStraight(0.3, 23, 0);
            liftBarUp();
            driveStraight(0.2, 7, 0);
            liftDown();
            driveStraight(0.3, -23, 0);
            turnToHeading(0.3, -90);
            driveStraight(0.3, 45, -90);

        }
        while (opModeIsActive()) {
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
