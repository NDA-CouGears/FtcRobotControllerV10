package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

//@Autonomous( name = "AutoMode Base", group = "OpModes")
public class AutoMode extends RobotParent {

    //@Juliet opens/closes claw and moves elbow arm up/down
    protected void closeClaw(){
        claw.setPosition(ClawClosed);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    protected void openClaw(){
        claw.setPosition(ClawOpen);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    protected void armDown(){
        arm.setPosition(ARM_DOWN);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    protected void armSetPosition(double amount, long delay){
        try {
            arm.setPosition(ARM_DOWN*amount);
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    protected void armBasketDown(){
        arm.setPosition(0.35);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
    protected void armUp(){
        arm.setPosition(ARM_UP);
    }
    //moveRobot moves the robot (shocking i know). used inside the driveForward and slide methods.
    public void turnToHeading(double maxTurnSpeed, double heading) {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
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
        //clear cache to get most recent encoder value(s)
        clearBulkCache();

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            ElapsedTime runtime = new ElapsedTime();
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

            // We are using bulk cache so the isBusy flag will be cached along with all other sensor
            // values when we accessed the encoder values above. Flush the cache no to ensure we get an
            // up to date reading.
            clearBulkCache();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                if (runtime.seconds() < 1){
                    moveRobot(maxDriveSpeed * runtime.seconds() * 2, 0, turnSpeed);
                }
                else {
                    double remainingDistance = (newLeftFrontTarget - leftFrontDrive.getCurrentPosition())/COUNTS_PER_INCH;
                    double driveSpeed = remainingDistance/10 * maxDriveSpeed;
                    driveSpeed = Range.clip(driveSpeed, -maxDriveSpeed, maxDriveSpeed);
                    moveRobot(driveSpeed, 0, turnSpeed);
                }


                // Display drive status for the driver.
                updateTelemetry();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);

        }
    }
    public void slide(double maxDriveSpeed,
                              double distance,
                              double heading) {

        int moveCounts = (int)(distance * SLIDE_COUNTS_PER_INCH);
        int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - moveCounts;
        int newLeftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
        int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
        int newRightBackTarget = rightBackDrive.getCurrentPosition() - moveCounts;

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

        // We are using bulk cache so the isBusy flag will be cached along with all other sensor
        // values when we accessed the encoder values above. Flush the cache no to ensure we get an
        // up to date reading.
        clearBulkCache();

        // keep looping while we are still active, and BOTH motors are running.
        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display drive status for the driver.
            updateTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0, 0);

    }
    public void liftLowerBasket(){
        armMotor.setTargetPosition(2000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        while (opModeIsActive() && armMotor.isBusy()) {
            idle();
        }
        armMotor.setPower(0);
    }
    public void liftHigherBasket(){
        armMotor.setTargetPosition(5000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

        while (opModeIsActive() && armMotor.isBusy()) {
            idle();
        }
        armMotor.setPower(0);
    }

    public void liftHigherBasketNoWait(){
        armMotor.setTargetPosition(5000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);

    }

    public void waitForLift(){
        while (opModeIsActive() && armMotor.isBusy()) {
            //idle();
        }
        armMotor.setPower(0);
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
        telemetry.addData("Left Front Motor", "%d, %d, %.2f", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getTargetPosition(), leftFrontDrive.getPower());
        telemetry.addData("Right Front Motor", "%d, %d, %.2f", rightFrontDrive.getCurrentPosition(), rightFrontDrive.getTargetPosition(), rightFrontDrive.getPower());
        telemetry.addData("Left Back Motor", "%d, %d, %.2f", leftBackDrive.getCurrentPosition(), leftBackDrive.getTargetPosition(), leftBackDrive.getPower());
        telemetry.addData("Right Back Motor", "%d, %d, %.2f", rightBackDrive.getCurrentPosition(), rightBackDrive.getTargetPosition(), rightBackDrive.getPower());
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {}
}
