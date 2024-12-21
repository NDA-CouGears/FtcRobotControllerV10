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
            liftHigherBasketNoWait();
            driveStraight(0.4, 4, 0);
            turnToHeading(0.4, 90);
            driveStraight(0.4, 12, 90);
            turnToHeading(0.4, 120);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            liftDownNoWait();
            //gets first block; puts in higher basket
            turnToHeading(0.4, -3);
            armSetPosition(.75, 0);
            driveStraight(0.4, 14, -3);
            turnToHeading(0.4, -3);
            armSetPosition(1,600);
            closeClaw();
            armUp();
            driveStraight(0.4, -13, -3);
            turnToHeading(0.4, 120);
            liftHigherBasketNoWait();
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            //gets second block; puts in higher basket
            turnToHeading(0.4, 0);
            armSetPosition(0.75, 0);
            driveStraight(0.4, 13, 0);
            slide(0.2, 9, 0);
            turnToHeading(0.4, 0);
            armSetPosition(1, 600);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            slide(0.2, -8, 0);
            driveStraight(0.4, -15, -8);
            turnToHeading(0.4, 125);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            turnToHeading(0.4,0);
            slide(0.4, -13.5,0);
            driveStraight(0.4, 49, 0);
            slide(0.4, -1,0);

            
        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
