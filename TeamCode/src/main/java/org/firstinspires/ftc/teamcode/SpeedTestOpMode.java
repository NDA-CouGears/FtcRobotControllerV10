package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Hardware Test", group="Tests")
public class SpeedTestOpMode  extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor lift;
    Servo claw;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        /////////////////////////////////////////////
        // OpMode initialization
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        double claw_pos = 0;
        claw = hardwareMap.get(Servo.class, "claw");
        //arm = hardwareMap.get(Servo.class, "arm");

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor. ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        /////////////////////////////////////////////
        // Initialization complete, wait for start
        telemetry.addData(">", "Press Start to run tests.");
        telemetry.update();
        waitForStart();

        /////////////////////////////////////////////
        // Run op mode
        runtime.reset();

        String runTime = runtime.toString();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Test Run Time: " + runTime);

            if (claw != null) {
                if (gamepad1.dpad_left) {
                    claw_pos -= 0.01;
                }
                if (gamepad1.dpad_right) {
                    claw_pos += 0.01;
                }
                if (claw_pos > 1) {
                    claw_pos = 1;
                }
                else if (claw_pos < 0) {
                    claw_pos = 0;
                }
                claw.setPosition(claw_pos);
            }

            if (lift != null) {
                double lift_delta = gamepad1.left_stick_y;

                lift.setPower(lift_delta);
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Lift position", lift.getCurrentPosition());
            telemetry.addData("Claw position", claw.getPosition());

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}
