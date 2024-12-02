import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous( name = "AutoRightPath", group = "OpModes")
public class AutoRightPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if(opModeIsActive()) {
            armUp();
            openClaw();
            driveStraight(0.3, 19, 0);
            slide(0.3, -20, 0);
            turnToHeading(0.2, 0);
            armDown();
            closeClaw();
            armUp();
            driveStraight(0.3, -17, 0);
            turnToHeading(0.3, -90);
            armDown();
            openClaw();
            armUp();
            turnToHeading(0.3, 0);
            driveStraight(0.3, 17, 0);
            slide(0.3, -10, 0);
            armDown();
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
