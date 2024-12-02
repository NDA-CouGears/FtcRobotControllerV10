import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous( name = "AutoMiddlePath", group = "OpModes")
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
            driveStraight(0.2, 2, 0);
            liftDown();
            driveStraight(0.3, -23, 0);
            turnToHeading(0.3, -90);
            driveStraight(0.3, 20, 0);
        }
        while (opModeIsActive()) {
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
