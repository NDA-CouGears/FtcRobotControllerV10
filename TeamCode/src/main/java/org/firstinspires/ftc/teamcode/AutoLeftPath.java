package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "LeftPath", group = "OpModes")
public class AutoLeftPath extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        robotInit();
        //waitForStart();
        imu.resetYaw();

        if (opModeIsActive()){
            //starts w/ block; puts in higher basket
            armUp();
            closeClaw();
            liftHigherBasketNoWait();
            slide(0.4, -4, 0);
            driveToDistance(0.6, 11.5, 0, 0.05);
            turnToHeading(0.4, 30);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(50);
            liftHigherBasketNoWait();
            //gets first block; puts in higher basket
            turnToHeading(0.4, -95);
            liftDownNoWait();
            armSetPosition(1, 600);
            driveToDistance(0.6, 20, 0, -95, true, false, 0.05);
            turnToHeading(0.4, -95);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            driveToDistance(0.6, 6.5, 0, -95, true, false, 0.05);
            turnToHeading(0.4, 30);
            waitForLift();
            armBasketDown();
            openClaw();
            armUp();
            sleep(100);
            //gets second block; puts in higher basket
            turnToHeading(0.4, -90);
            liftDownNoWait();
            armSetPosition(0.9, 600);
            driveToDistance(0.6, 19.5, 0, -90, true, false, 0.05);
            driveToDistance(0.6, 0, 2, -90, false, true, 0.05);
            armSetPosition(1, 500);
            turnToHeading(0.4, -90);
            closeClaw();
            armUp();
            liftHigherBasketNoWait();
            slide(0.2, -8, -90);
            driveStraight(0.4, -14, -98);
            turnToHeading(0.4, 35);
            waitForLift();
            armBasketDown();
            halfOpenClaw();
            armUp();
            sleep(100);
            turnToHeading(0.4, 0);
            liftDownNoWait();
            //gets third block and puts in higher basket
            driveToDistance(0.6, 6, 18.5, 0, false, true, 0.05);
            turnToHeading(0.4, -55);
            armDown();
            driveStraight(0.2, 2.5, -55);
            closeClaw();
            armUp();
            driveStraight(0.6, -10, -55);
            liftHigherBasketNoWait();
            turnToHeading(0.5, 50);
            driveStraight(0.4, 3, 55);
            armBasketDown();
            openClaw();
            armUp();
            //backs up
            driveStraight(0.5, -10, 55);
            
        }
    }
}
