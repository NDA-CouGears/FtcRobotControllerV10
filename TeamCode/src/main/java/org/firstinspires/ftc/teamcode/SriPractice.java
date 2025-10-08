package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
class MotorGroup {
    DcMotor a;
    DcMotor b;
    DcMotor c;
    DcMotor d;

    MotorGroup(DcMotor a,DcMotor b,DcMotor c,DcMotor d) {
        this.a=a;
        this.b=b;
        this.c=c;
        this.d=d;
    }
}
*/
@TeleOp(name = "SriOpMode")

public class SriPractice extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DistanceSensor frontSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class,"lf_drive");
        frontRight = hardwareMap.get(DcMotor.class,"rf_drive");
        backLeft = hardwareMap.get(DcMotor.class,"lb_drive");
        backRight = hardwareMap.get(DcMotor.class,"rb_drive");
        frontSensor = hardwareMap.get(DistanceSensor.class,"distance");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            //double forward = -(gamepad1.left_stick_y);
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            if (gamepad1.x) {
                double target = 15;
                while (opModeIsActive()) {
                    double dist = frontSensor.getDistance(DistanceUnit.INCH);
                    double error = dist - target;
                    double power = error / 24;
                    frontLeft.setPower(power);
                    frontRight.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(power);
                    if (Math.abs(error) < 1)
                        break;
                }
            }
        }
    }
}
