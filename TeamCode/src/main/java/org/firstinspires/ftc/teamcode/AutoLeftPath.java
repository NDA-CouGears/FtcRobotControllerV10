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
            driveToDistance(0.6, 11.5, 0, 0.05);
            turnToHeading(0.4, 30);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            liftDownNoWait();
            //gets first block; puts in higher basket
            turnToHeading(0.4, -95);
            armSetPosition(1, 600);
            //driveStraight(0.4, 14, -93);
            driveToDistance(0.6, 20.5, 0, -95, true, false, 0.05);
            turnToHeading(0.4, -95);
            //armSetPosition(1,3000);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            driveToDistance(0.6, 7.5, 0, -95, true, false, 0.05);
            turnToHeading(0.4, 30);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            //gets second block; puts in higher basket
            turnToHeading(0.4, -90);
            armSetPosition(1, 600);
            driveToDistance(0.6, 17.5, 0, -90, true, false, 0.05);
            //driveStraight(0.4, 13, -90);
            //slide(0.2, 9, -90);
            driveToDistance(0.6, 0, 2, -90, false, true, 0.05);
            turnToHeading(0.4, -90);
            //armSetPosition(1, 600);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            slide(0.2, -8, -90);
            driveStraight(0.4, -14.5, -98);
            turnToHeading(0.4, 35);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            liftDownNoWait();
            turnToHeading(0.4, 0);
            driveToDistance(0.6, 6, 18.5, 0, false, true, 0.05);
            turnToHeading(0.4, -55);
            armDown();
            driveStraight(0.2, 3, -55);
            closeClaw();
            armUp();
            driveToDistance(0.3, 12, 6, -90, );

            /*
            turnToHeading(0.4,-90);
            slide(0.4, -13.5,-90);
            driveStraight(0.4, 49, -90);
            slide(0.4, -1,-90);

             */

            
        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }
    }
}
