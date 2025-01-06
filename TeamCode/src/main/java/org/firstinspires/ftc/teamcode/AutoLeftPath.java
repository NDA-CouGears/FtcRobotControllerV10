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
            slide(0.4, -4, 0);
            //driveStraight(0.4, 4, 0);
            //turnToHeading(0.4, 90);
            driveToDistance(0.4, 11.5, 0);
            turnToHeading(0.4, 30);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            liftDownNoWait();
            //gets first block; puts in higher basket
            turnToHeading(0.4, -93);
            armSetPosition(1, 600);
            driveStraight(0.4, 14, -93);
            turnToHeading(0.4, -93);
            //armSetPosition(1,3000);
            closeClaw();
            armUp();
            driveStraight(0.4, -13, -93);
            turnToHeading(0.4, 30);
            liftHigherBasketNoWait();
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            //gets second block; puts in higher basket
            turnToHeading(0.4, -90);
            armSetPosition(0.75, 500);
            driveStraight(0.4, 13, -90);
            slide(0.2, 9, -90);
            turnToHeading(0.4, -90);
            armSetPosition(1, 600);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            slide(0.2, -8, -90);
            driveStraight(0.4, -15, -98);
            turnToHeading(0.4, 35);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            turnToHeading(0.4,-90);
            slide(0.4, -13.5,-90);
            driveStraight(0.4, 49, -90);
            slide(0.4, -1,-90);

            
        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
