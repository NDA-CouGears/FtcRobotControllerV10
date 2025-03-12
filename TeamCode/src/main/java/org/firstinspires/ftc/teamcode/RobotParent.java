package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

    protected DcMotor climbMotor;
    protected CRServo climbServo;

    protected TouchSensor touchSensor = null;

    protected DistanceSensor sensorFrontDistance = null;
    protected DistanceSensor sensorLeftDistance = null;
    protected DistanceSensor sensorRightDistance = null;
    protected DistanceSensor sensorBackDistance = null;


    final protected static double ClawClosed = 0.57;
    final protected static double ClawOpen = 0.22;

    protected Servo secondArm = null; //this arm is the one that picks up specimens

    public static double ARM_UP = 0.24;
    public static double ARM_DOWN = 0.73;

    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 102 / 25.4;
    static final double COUNTS_PER_MOTOR_REV = 483.3836858;  //NEED TO FIX DIS >:3
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.2;
    static final double SLIDE_COUNTS_PER_INCH = COUNTS_PER_INCH / 0.7251;

    public double targetHeading = 0;
    public double headingError = 0;
    public double turnSpeed = 0;

    public IMU imu = null;// Control/Expansion Hub IMU

    static final double P_DRIVE_GAIN = 0.03;// Larger is more responsive, but also less stable.
    static final double P_TURN_GAIN = 0.02;// Larger is more responsive, but also less stable.

    static final double HEADING_THRESHOLD = 1.0;// How close must the heading get to the target before moving to next step.


    private double signPreserveSquare(double value) {

        if (value > 0) {
            return value * value;
        } else {
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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
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

    public void climb() {
        double climbDelta = gamepad2.left_stick_y;

        if (climbDelta > 0)
            climbServo.setPower(climbDelta);
        else
            climbServo.setPower(0);

        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if( climbMotor.getCurrentPosition() > -2700 || climbDelta < 0 )
            climbMotor.setPower(climbDelta * -0.25);
        else
            climbMotor.setPower(0);

        telemetry.addData("Climb motor pos", "%d ", climbMotor.getCurrentPosition() );

        // Lock the climb at a fixed height when requested, this is for the end game so we
        // lock out all other actions and block here
        if (gamepad2.x) {
            int targetHeight = climbMotor.getCurrentPosition();

            while (opModeIsActive()) {
                int curHeight = climbMotor.getCurrentPosition();
                if (curHeight < targetHeight) {
                    climbMotor.setPower(0.25);
                }
                else {
                    climbMotor.setPower(0);
                }
                if (gamepad2.y) {
                    break;
                }
            }

        }
    }

    public void clearBulkCache() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees to avoid wasting time with overly long turns
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void driveToDistance(double maxSpeed, double targetDistance, double heading) {
        driveToDistance(maxSpeed, targetDistance, 0, heading, false, false, 0.03);
    }
    public void driveToDistance(double maxSpeed, double targetDistance, double heading, double speedGain) {
        driveToDistance(maxSpeed, targetDistance, 0, heading, false, false, speedGain);
    }
    public void driveToDistance(double maxSpeed, double targetForwardDistance, double targetLateralDistance, double heading, boolean backwards, boolean left) {
        driveToDistance(maxSpeed, targetForwardDistance, targetLateralDistance, heading, backwards, left, 0.03);
    }
    public void driveToDistance(double maxSpeed, double targetForwardDistance, double targetLateralDistance, double heading, boolean backwards, boolean left, double speedGain) {
        clearBulkCache();
        //final double SPEED_GAIN = 0.03;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        DistanceSensor localForwardSensor = backwards ? sensorBackDistance : sensorFrontDistance;
        DistanceSensor localLateralSensor = left ? sensorLeftDistance : sensorRightDistance;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        do {
            while (opModeIsActive() && !gamepad1.y) {

                double rangeErrorForward = targetForwardDistance == 0 ? 0
                        : (localForwardSensor.getDistance(DistanceUnit.INCH) - targetForwardDistance);
                double rangeErrorLateral = targetLateralDistance == 0 ? 0
                        : (localLateralSensor.getDistance(DistanceUnit.INCH) - targetLateralDistance);

                // If we are close on all axes stop, we need to experiment to find good values
                if (Math.abs(rangeErrorForward) < 1 && Math.abs(rangeErrorLateral) < 1) {
                    break;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move. These are
                // more values with best guesses that need experimentation to find good values
                double forwardDriveSpeed = 0;
                double lateralDriveSpeed = 0;
                double turnSpeed = getSteeringCorrection(heading, TURN_GAIN);

                if ( rangeErrorForward < 0 ) {
                    forwardDriveSpeed = Range.clip(rangeErrorForward * speedGain, -maxSpeed, -0.1);
                }
                else if ( rangeErrorForward > 0 ) {
                    forwardDriveSpeed = Range.clip(rangeErrorForward * speedGain, 0.1, maxSpeed);
                }
                if ( rangeErrorLateral < 0 ) {
                    lateralDriveSpeed = Range.clip(rangeErrorLateral * speedGain, -maxSpeed, -0.1);
                } else if ( rangeErrorLateral > 0 ){
                    lateralDriveSpeed = Range.clip(rangeErrorLateral * speedGain, 0.1, maxSpeed);
                }
                telemetry.addData("Auto DTD", "Drive %5.2f Lateral %5.2f Turn %5.2f ", forwardDriveSpeed, lateralDriveSpeed, turnSpeed);

                // For debugging let us pause motion to see telemetry
                if (gamepad1.y) {
                    moveRobot(0, 0, 0);
                } else {
                    moveRobot(backwards ? -forwardDriveSpeed : forwardDriveSpeed,
                            left ? lateralDriveSpeed : -lateralDriveSpeed, turnSpeed);
                }

                checkSensor(); //prevent the arm from going BZZRRRR

                telemetry.update();
            }
//        } while (opModeIsActive() && localForwardSensor.getDistance(DistanceUnit.INCH) > targetForwardDistance && !gamepad1.y); // if we over shot loop again to back up a bit

        moveRobot(0, 0, 0);
    }

    public void liftBarUp() {
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        clearBulkCache();
        while (opModeIsActive() && armMotor.isBusy() && !gamepad1.y) {
            idle();
        }
        armMotor.setPower(0);
    }

    public void liftBarUpNoWait() {
        armMotor.setTargetPosition(1000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void liftDown() {
        armMotor.setTargetPosition(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        while (opModeIsActive() && armMotor.isBusy()) {
            checkSensor();
        }
        armMotor.setPower(0);
    }

    public void liftDownNoWait() {
        armMotor.setTargetPosition(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void robotInit(){
        while (opModeInInit()){
            telemetry.addData("Front", "%2.2f", sensorFrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Back", "%2.2f", sensorBackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left", "%2.2f", sensorLeftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right", "%2.2f", sensorRightDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    public void checkSensor(){
        if ((touchSensor.isPressed()) && (armMotor.getPower() < 0)) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setPower(0);
        }
    }


    /**
     * <Robot type="FirstInspires-FTC">
     * <LynxUsbDevice name="Control Hub Portal" serialNumber="(embedded)" parentModuleAddress="173">
     * <LynxModule name="Expansion Hub 2" port="2">
     * <goBILDA5202SeriesMotor name="lift" port="0"/>
     * <goBILDA5202SeriesMotor name="climb" port="1"/>
     * <REV_VL53L0X_RANGE_SENSOR name="distance" port="0" bus="0"/>
     * <REV_VL53L0X_RANGE_SENSOR name="left_sensor" port="0" bus="1"/>
     * <REV_VL53L0X_RANGE_SENSOR name="back_sensor" port="0" bus="2"/>
     * </LynxModule>
     * <LynxModule name="Control Hub" port="173">
     * <NeveRest20Gearmotor name="lb_drive" port="0"/>
     * <NeveRest20Gearmotor name="rb_drive" port="1"/>
     * <NeveRest20Gearmotor name="lf_drive" port="2"/>
     * <NeveRest20Gearmotor name="rf_drive" port="3"/>
     * <Servo name="claw" port="0"/>
     * <Servo name="arm" port="1"/>
     * <RevTouchSensor name="touchSensor" port="1"/>
     * <AdafruitBNO055IMU name="imu" port="0" bus="1"/>
     * <REV_VL53L0X_RANGE_SENSOR name="right_sensor" port="0" bus="2"/>
     * </LynxModule>
     * </LynxUsbDevice>
     * </Robot>
     */
    public void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_drive"); // control hub 2
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive"); // control hub 0
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive"); // control hub 3
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive"); // control hub 1

        claw = hardwareMap.get(Servo.class, "claw"); //  control hub 0
        arm = hardwareMap.get(Servo.class, "arm"); // control hub 1
        arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(ARM_UP);

        armMotor = hardwareMap.get(DcMotor.class, "lift"); // Expansion hub port 0

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor"); // control hub digital 1

        sensorFrontDistance = hardwareMap.get(DistanceSensor.class, "distance"); // expansion i2c 0
        sensorLeftDistance = hardwareMap.get(DistanceSensor.class, "left_sensor"); // expansion i2c 1
        sensorRightDistance = hardwareMap.get(DistanceSensor.class, "right_sensor"); // control hub bus 2
        sensorBackDistance = hardwareMap.get(DistanceSensor.class, "back_sensor"); // expansion i2c 2


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setDirection(Servo.Direction.FORWARD);

        climbMotor = hardwareMap.get(DcMotor.class, "climb");
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        climbServo = hardwareMap.get(CRServo.class, "climb_up");
        /* In case we need to customize pwm range
        ServoControllerEx controller = (ServoControllerEx) climbServo.getController();
        int portNum = climbServo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(553,2500);
        controller.setServoPwmRange(portNum,range);
        */
        climbServo.setDirection(DcMotorSimple.Direction.FORWARD);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
