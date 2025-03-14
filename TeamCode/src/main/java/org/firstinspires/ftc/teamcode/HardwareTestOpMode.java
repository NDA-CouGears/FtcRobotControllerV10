package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Disabled
@TeleOp(name="Hardware Test", group="Tests")
public class HardwareTestOpMode extends LinearOpMode {
    public static double ARM_UP = 0.24;
    public static double ARM_DOWN = 0.615;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor climbMotor;
    CRServo climbServo;

    DcMotor lift;
    Servo claw;
    Servo arm;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        /////////////////////////////////////////////
        // OpMode initialization
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        climbMotor = hardwareMap.tryGet(DcMotor.class, "climb");
        if (climbMotor != null) {
            climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        climbServo = hardwareMap.tryGet(CRServo.class, "climb_up");
        if (climbServo != null) {
            /* In case we need to customize pwm range
            ServoControllerEx controller = (ServoControllerEx) climb_servo.getController();
            int portNum = climb_servo.getPortNumber();
            PwmControl.PwmRange range = new PwmControl.PwmRange(553,2500);
            controller.setServoPwmRange(portNum,range);
            */
            climbServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        }

        double claw_pos = 0;
        claw = hardwareMap.get(Servo.class, "claw");

        double arm_pos = ARM_UP;
        arm = hardwareMap.tryGet(Servo.class, "arm");
        if (arm != null) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(arm_pos);
        }

        lift = hardwareMap.get(DcMotor.class, "lift");
        int lift_hold_pos = 0;
        boolean lift_holding = false;
        if (lift != null) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

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

            if (arm != null) {
                if (gamepad1.dpad_down) {
                    arm_pos -= 0.005;
                }
                else if (gamepad1.dpad_up) {
                    arm_pos += 0.005;
                }
                else if (gamepad1.left_bumper) {
                    arm_pos = ARM_UP;
                }
                else if (gamepad1.right_bumper) {
                    arm_pos = ARM_DOWN;
                }
                if (arm_pos > 1) {
                    arm_pos = 1;
                }
                else if (arm_pos < 0) {
                    arm_pos = 0;
                }
                arm.setPosition(arm_pos);
            }

            if (lift != null) {
                if (Math.abs(gamepad1.left_stick_y) < 0.01) {
                    lift.setTargetPosition(lift_hold_pos);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);

                    while (opModeIsActive() && lift.isBusy() && (Math.abs(gamepad1.left_stick_y) < 0.01)) {
                        idle();
                    }

                    lift.setPower(0);
                }
                else {
                    double lift_delta = -gamepad1.left_stick_y;
                    lift_hold_pos = lift.getCurrentPosition();
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(lift_delta);
                }
            }

            if (climbMotor != null) {
                double climb_delta = gamepad2.left_stick_y;
                if (climbServo != null) {
                    if (climb_delta > 0)
                        climbServo.setPower(climb_delta);
                    else
                        climbServo.setPower(0);
                }
                climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                climbMotor.setPower(climb_delta*-0.25);
            }

            if (imu != null) {
                if (gamepad1.y) {
                    imu.resetYaw();
                }
            }

            if (lift != null)
                telemetry.addData("Lift position", lift.getCurrentPosition());
            if (claw != null)
                telemetry.addData("Claw position", claw.getPosition());
            if (arm != null)
                telemetry.addData("Arm position", arm.getPosition());

            if (imu != null) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            }

            if(climbMotor != null){
                telemetry.addData("Climb motor pos", "%d ", climbMotor.getCurrentPosition() );
            }
            telemetry.update();
        }
    }
}
