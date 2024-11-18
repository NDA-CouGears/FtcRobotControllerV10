package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public abstract class RobotParent extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected Servo claw = null;
    protected Servo arm = null;

    protected DcMotor armMotor = null;

    protected TouchSensor touchSensor = null;

    final protected static double ClawClosed = 0.23;
    final protected static double ClawOpen = 0.92;

    protected Servo secondArm = null; //this arm is the one that picks up specimens

    public static double ARM_UP = 0.24;
    public static double ARM_DOWN = 0.615;

    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_MM = 102;
    static final double COUNTS_PER_MOTOR_REV = 0;  //NEED TO FIX DIS >:3
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.14159);
    static final double DRIVE_SPEED = 0.6;


    private double signPreserveSquare(double value) {

        if (value > 0) {
            return value * value;
        }
        else {
            return -(value * value);
        }
    }

    protected void encoderDrive(double speed,
                                double leftFrontInches,
                                double rightFrontInches,
                                double leftBackInches,
                                double rightBackInches,
                                double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        speed = 0.25;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (25.4 * leftFrontInches * COUNTS_PER_MM);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (25.4 * leftBackInches * COUNTS_PER_MM);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (25.4 * rightFrontInches * COUNTS_PER_MM);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int) (25.4 * rightBackInches * COUNTS_PER_MM);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Currently at", " at %7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            telemetry.addData("Moved to", " %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
            telemetry.update();

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void mDrive(){
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = signPreserveSquare(gamepad1.left_stick_y*-0.9); // Remember, this is reversed!
        double lateral = signPreserveSquare(gamepad1.left_stick_x * 0.7); // Counteract imperfect strafing
        double yaw = (signPreserveSquare(gamepad1.right_stick_x * 1))*0.5;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        int lfp = leftFrontDrive.getCurrentPosition();
        int rfp = rightFrontDrive.getCurrentPosition();
        int lbp = leftBackDrive.getCurrentPosition();
        int rbp = rightBackDrive.getCurrentPosition();
    }

    protected void arm(){
        boolean holdingAtA = false;
        boolean holdingAtB = false;
        if (gamepad2.y){
            holdingAtA = false;
            holdingAtB = false;
        }
        else if ((gamepad2.a) || (holdingAtA)) {
            armMotor.setTargetPosition(2000);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);

            while (opModeIsActive() && armMotor.isBusy() && (!gamepad2.y)) {
                idle();
            }

            armMotor.setPower(0);
            if (!gamepad2.y) {
                holdingAtA = true;
            }
        }
        else if ((gamepad2.b) || (holdingAtB)) {
            armMotor.setTargetPosition(4700);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);

            while (opModeIsActive() && armMotor.isBusy() && (!gamepad2.y)) {
                idle();
            }

            armMotor.setPower(0);
            if (!gamepad2.y){
                holdingAtB = true;
            }
        }
        else {
            double armMotorPower = -gamepad2.right_stick_y;
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if ((touchSensor.isPressed()) && (armMotorPower < 0)) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            else {
                armMotor.setPower(armMotorPower);
            }

            telemetry.addData("Motor power: ", armMotor.getPower());
            telemetry.addData("Motor position: ", armMotor.getCurrentPosition());
        }
    }

    protected void claw(){
        //pressing right bumper opens claw, left bumper closes claw
        if (gamepad2.right_bumper){
            claw.setPosition(ClawOpen);
        }

        else if (gamepad2.left_bumper){
            claw.setPosition(ClawClosed);
        }
    }
    public void initHardware(){
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        armMotor = hardwareMap.get(DcMotor.class, "lift");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setDirection(Servo.Direction.FORWARD);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
