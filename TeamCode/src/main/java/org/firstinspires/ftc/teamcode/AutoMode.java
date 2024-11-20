package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous( name = "AutoMode Base", group = "OpModes")
public class AutoMode extends RobotParent {


    /* @Tess. These are all the methods for driving, slide just means to go sideways.
    For doing any forward movement, just init a new encoderDrive and manually put in the
    the inches you want to travel. ( DRIVE_SPEED and timeout are the same :3 )
    */
    private void slideLeft (int direction, int inches){
        encoderDrive(DRIVE_SPEED, -inches , inches, inches, -inches, 10.0);
    }
    private void slideRight (int direction, int inches){
        encoderDrive(DRIVE_SPEED, inches , -inches, -inches, inches, 10.0);
    }
    private void turnLeft (int direction, int inches){
        encoderDrive(DRIVE_SPEED, -inches , inches, inches, -inches, 10.0);
    }
    private void turnRight (int direction, int inches) {
        encoderDrive(DRIVE_SPEED, inches , -inches, -inches, inches, 10.0);
    }
    //@Juliet opens/closes claw and moves elbow arm up/down
    private void closeClaw(){
        claw.setPosition(ClawClosed);
    }
    private void openClaw(){
        claw.setPosition(ClawOpen);
    }
    private void armDown(){
        arm.setPosition(ARM_DOWN);
    }
    private void armUp(){
        arm.setPosition(ARM_UP);
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
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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
    public void turnToHeading(double maxTurnSpeed, double heading) {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            //GETtING STUCK IN LOOP B/C NOT REACHING ANGLE BC SPEED TOO SLOW
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            if( turnSpeed < 0 ) {
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, -0.1);
            }
            else{
                turnSpeed = Range.clip(turnSpeed,0.1, maxTurnSpeed);
            }
            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);
            updateTelemetry();
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            int newLeftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            int newRightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, 0, turnSpeed);

                // Display drive status for the driver.
                updateTelemetry();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void updateTelemetry(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.addData("Status", "Initialized2");
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if(opModeIsActive()) {
            //have the robot move at a higher speed first, then run a second method to correct position if overshot
            turnToHeading(1.0, -75);
            turnToHeading(0.2, -90);
            driveStraight(0.2, 30, -90);
        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }


    }
}
