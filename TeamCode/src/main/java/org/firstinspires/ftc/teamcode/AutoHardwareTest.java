package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous( name = "Hardware Auto Test", group = "OpModes")
public class AutoHardwareTest extends AutoMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()){
            if (gamepad1.a) {
                driveStraight(0.6, 40, 0);
                driveStraight(0.6, -40, 0);
            }
            else if (gamepad1.b) {
                slide(0.6, 40, 0);
                slide(0.6, -40, 0);
            }
        }
    }
}
