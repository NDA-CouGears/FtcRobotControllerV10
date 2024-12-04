package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "LeftPath", group = "OpModes")
public class AutoLeftPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()){
            armUp();
            closeClaw();
            driveStraight(0.4, 4, 0);
            turnToHeading(0.4, 90);
            liftHigherBasketNoWait();
            driveStraight(0.4, 12, 90);
            turnToHeading(0.4, 120);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            liftDownNoWait();

            turnToHeading(0.4, 0);
            driveStraight(0.4, 13, 0);
            turnToHeading(0.4, 0);
            waitForLift();
            armDown();
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            driveStraight(0.4, -13, 0);
            turnToHeading(0.4, 120);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();

            turnToHeading(0.4, 0);
            driveStraight(0.4, 13, 0);
            slide(0.4, 8, 0);
            armDown();
            closeClaw();
            armUp();
            slide(0.4, -8, 0);
            driveStraight(0.4, -13, 0);
            turnToHeading(0.4, 120);
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
