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
            //starts w/ block; puts in higher basket
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
            //gets first block; puts in higher basket
            turnToHeading(0.4, -3);
            driveStraight(0.4, 15, -3);
            armDown();
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            driveStraight(0.4, -14, -3);
            turnToHeading(0.4, 120);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            //gets second block; puts in higher basket
            turnToHeading(0.4, 0);
            driveStraight(0.4, 13, 0);
            slide(0.2, 8, 0);
            armDown();
            closeClaw();
            armUp();
            slide(0.2, -8, 0);
            liftHigherBasketNoWait();
            driveStraight(0.4, -13, -8);
            turnToHeading(0.4, 125);
            waitForLift();
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
