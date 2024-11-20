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

        // Normalize the error to be within +/- 180 degrees
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

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            //GETIING STUCK IN LOOP B/C NOT REACHING ANGLE BC SPEED TOO SLOW
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);
            updateTelemetry();
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
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

    /* @Tess. These are the 90 turning methods from last year, I will probably have to change
    the encoder drive values later to account for the new wheels, so this is commented out for now.

    protected void turnLeft90(){
        encoderDrive(DRIVE_SPEED,   -17.5, 17.5, -17.5, 17.5,4.0);
    }
    protected void turnRight90(){
        encoderDrive(DRIVE_SPEED,   17.5, -17.5, 17.5, -17.5,4.0);
    }
    */
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        imu.resetYaw();

        if(opModeIsActive()) {
            turnToHeading(0.2, -90);
        }
        while (opModeIsActive()){
            telemetry.addLine("after turn");
            updateTelemetry();
        }


    }
}
