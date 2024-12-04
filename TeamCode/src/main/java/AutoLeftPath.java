import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous( name = "AutoLeftPath", group = "OpModes")
public class AutoLeftPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()){
            armUp();
            closeClaw();
            driveStraight(0.3, 4, 0);
            turnToHeading(0.3, 90);
            driveStraight(0.3, 12, 90);
            turnToHeading(0.3, 120);
            liftHigherBasket();
            armBasketDown();
            openClaw();
            armUp();
            liftDown();

            turnToHeading(0.3, 0);
            driveStraight(0.3, 13, 0);
            armDown();
            closeClaw();
            armUp();
            driveStraight(0.3, -13, 0);
            turnToHeading(0.3, 120);
            liftHigherBasket();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDown();



        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
