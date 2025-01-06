package org.firstinspires.ftc.teamcode;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp( name = "Vision", group = "Examples")
public class VisionExampleOpMode extends AutoMode {
    private WebcamName forward_cam, downward_cam;
    VisionPortal portal;
    AprilTagProcessor tagProcessor;
    ColorBlobLocatorProcessor yellowLocator;
    ColorBlobLocatorProcessor redLocator;
    ColorBlobLocatorProcessor blueLocator;

    public void initVision() {
        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Turn this off for competitions to save cpu
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Turn this off for competitions to save cpu
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Turn this off for competitions to save cpu
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        // Create the AprilTag processor by using a builder.
        tagProcessor = new AprilTagProcessor.Builder().build();
        // We are only using large april tags this year so we can trade performance (higher
        // decimation) for accuracy from range. With the smaller april tags we would lower this.
        tagProcessor.setDecimation(3);

        // Ground level cam aimed straight ahead, good for april tags
        forward_cam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Elevated camera angled downward, good for sample location and identification
        downward_cam = hardwareMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(downward_cam, forward_cam);

        portal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(blueLocator, redLocator, yellowLocator, tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (portal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        //waitForStart();

        // While in init offer some telemetry to help debug vision issues
        while (opModeInInit()) {
            debugVision();
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            telemetry.addLine("Use joysticks to drive until a tag or blob is in view");
            telemetry.addLine("Press DPad Left to drive to april tag");
            telemetry.addLine("Press DPad Right to drive to largest blob");

            if (gamepad1.dpad_left) {
                // When looking for april tags switch to forward camera
                portal.setActiveCamera(forward_cam);
                // This was copied from the example but was causing issues, need to look into it. We
                // Do seem to lose tracking while in motion so we should try to find values that
                // improve that
                //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

                // Drive to the tag in front of us until we are 18 inches away
                driveToTag(18);
            }
            else if (gamepad1.dpad_right) {
                // When looking for blobs switch to downward camera
                portal.setActiveCamera(downward_cam);

                // Current camera location dictate the target x and y for blob, this is pixel
                // location in the camera image of the desired center of the blob. Maintain our
                // current heading while lining up with the target
                driveToBlob(0.2,400, 50, getHeading());
            }
            else if (gamepad1.dpad_up) {
                driveToDistance(1,7, getHeading());
            }
            else {
                debugVision();

                double x = -gamepad1.left_stick_y * .5;
                double y = -gamepad1.left_stick_x * .5;
                double yaw = -gamepad1.right_stick_x;
                moveRobot(x,y,yaw);
            }

            telemetry.update();
        }

        portal.close();
    }

    public void driveToDistance(double maxSpeed, double targetDistance, double heading) {
        final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        do {
            while (opModeIsActive()) {
                double rangeError = (sensorDistance.getDistance(DistanceUnit.INCH) - targetDistance);

                // If we are close on all axes stop, we need to experiment to find good values
                if (Math.abs(rangeError) < 1) {
                    break;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move. These are
                // more values with best guesses that need experimentation to find good values
                double driveSpeed = 0;
                double turnSpeed = getSteeringCorrection(heading, TURN_GAIN);

                if (rangeError < 0) {
                    driveSpeed = Range.clip(rangeError * SPEED_GAIN, -maxSpeed, -0.1);
                } else {
                    driveSpeed = Range.clip(rangeError * SPEED_GAIN, 0.1, maxSpeed);
                }
                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f ", driveSpeed, turnSpeed);

                // For debugging let us pause motion to see telemetry
                if (gamepad1.y) {
                    moveRobot(0, 0, 0);
                } else {
                    moveRobot(driveSpeed, 0, turnSpeed);
                }

                telemetry.update();
            }
        } while (sensorDistance.getDistance(DistanceUnit.INCH) > targetDistance); // if we over shot loop again to back up a bit

        moveRobot(0, 0, 0);
    }

    private void driveToTag(double tagTargetDistance) {
        // Modified code taken from RobotAutoDriveToAprilTagOmni
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        while (opModeIsActive())
        {
            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            if (currentDetections.isEmpty()) {
                // no tag to drive to, just return
                return;
            }

            // This is just a test, drive to the first april tag we see
            AprilTagDetection desiredTag  = currentDetections.get(0);

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to
            // control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - tagTargetDistance);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // If we are close on all axes stop, we need to experiment to find good values
            if (Math.abs(rangeError) < 1 && headingError < 1 && yawError < 1) {
                break;
            }

            // Use the speed and turn "gains" to calculate how we want the robot to move. These are
            // more values with best guesses that need experimentation to find good values
            // May need to add a minimum power check like in turnToHeading
            double driveSpeed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafeSpeed = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double turnSpeed = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            telemetry.addData("Auto Drive Distance","Range %f, Drive %5.2f, Strafe %5.2f, Turn %5.2f ", sensorDistance.getDistance(DistanceUnit.INCH), driveSpeed, strafeSpeed, turnSpeed);

            // For debugging let us pause motion to see telemetry
            if (gamepad1.y) {
                moveRobot(0, 0, 0);
            }
            else {
                moveRobot(driveSpeed, strafeSpeed, turnSpeed);
            }

            telemetry.update();
        }

        moveRobot(0, 0, 0);
    }

    /**
     * Move the robot until the largest blob is located at x,y in the camera image.
     *
     * @param targetX The desired x location of the center of the blob
     * @param targetY The desired y location of the center of the blob
     */
    private void driveToBlob(double maxSpeed, double targetX, double targetY, double heading) {
        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = blueLocator.getBlobs();
            blobs.addAll(yellowLocator.getBlobs());
            blobs.addAll(redLocator.getBlobs());
            // More experimental numbers we need to tune
            ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, blobs);
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            if (blobs.isEmpty()) {
                // No blob in sight, return
                telemetry.addLine("No blob found!");
                return;
            }

            // Find the center of the largest blob
            ColorBlobLocatorProcessor.Blob target = blobs.get(0);
            Point center = target.getBoxFit().center;

            // Because the camera is pointing down we can use the y image location as a proxy
            // for forward distance and the x for horizontal distance. We depend on the caller
            // to provide a desired heading
            double  rangeError = (center.y - targetY);
            double  strafeError = (center.x - targetX);
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            telemetry.addLine(String.format(Locale.US,
                    "Driving to: (%3d,%3d) %5d  %4.2f   %5.2f",
                    (int) center.x, (int) center.y, target.getContourArea(), target.getDensity(), target.getAspectRatio()));
            telemetry.addLine(String.format(Locale.US,
                    "Errors (range, strafe, turn): %3.2f, %3.2f, %3.2f",rangeError, strafeError, headingError));

            // To avoid wiggling when the pixel error gets small zero it out
            if (Math.abs(rangeError) < 5) {
                rangeError = 0;
            }
            if (Math.abs(strafeError) < 5) {
                strafeError = 0;
            }

            // If we are close on all axes stop, we need to experiment to find good values, too small
            // and we will loop forever trying to get perfect, too large and we will miss our target
            // Keep in mind that range and strafe error are in pixels, headingError is in degrees
            if (rangeError == 0 && strafeError == 0 && Math.abs(headingError) < HEADING_THRESHOLD) {
                break;
            }

            double driveSpeed  = Range.clip(rangeError * 0.01, -maxSpeed, maxSpeed);
            double strafeSpeed = Range.clip(strafeError * 0.01, -maxSpeed, maxSpeed);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveSpeed, strafeSpeed, turnSpeed);

            // For debugging let us pause motion to see telemetry
            if (gamepad1.y) {
                moveRobot(0, 0, 0);
            }
            else {
                moveRobot(driveSpeed, strafeSpeed, turnSpeed);
            }

            telemetry.update();
        }

        moveRobot(0, 0, 0);
    }

    /**
     * Display telemetry of what the cameras are seeing for april tags and blobs. Note we have two
     * cameras so there are control to switch between them for testing.
     */
    private void debugVision() {
        telemetry.addLine("Press A to use forward camera");
        telemetry.addLine("Press B to use downward camera");

        // Turn stream on or off
        if (gamepad1.back && (portal.getCameraState() == VisionPortal.CameraState.STREAMING)) {
            portal.stopStreaming();
        } else if (gamepad1.start && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            portal.resumeStreaming();
        }

        // Don't bother looking if the portal is not processing video
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        // Switch between cameras if requested
        if (gamepad1.a && (portal.getActiveCamera() == downward_cam)) {
            portal.setActiveCamera(forward_cam);
        } else if (gamepad1.b && (portal.getActiveCamera() == forward_cam)) {
            portal.setActiveCamera(downward_cam);
        }

        List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> redBlobs = redLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blueBlobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, redBlobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, yellowBlobs);  // filter out very small blobs.
        //ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default so no need to call

        telemetry.addLine("Found id, range, bearing, yaw");
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US,"%d (%s), %5.1f, %3.0f, %3.0f",
                        detection.id, detection.metadata.name, detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
            }
        }

        telemetry.addLine("(x,y) Area Density Aspect");

        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob b : blueBlobs) {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format(Locale.US,
                    "Blue: (%3d,%3d) %5d  %4.2f   %5.2f",
                    (int) boxFit.center.x, (int) boxFit.center.y, b.getContourArea(), b.getDensity(), b.getAspectRatio()));
        }
        for (ColorBlobLocatorProcessor.Blob b : redBlobs) {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format(Locale.US,
                    "Red: (%3d,%3d) %5d  %4.2f   %5.2f",
                    (int) boxFit.center.x, (int) boxFit.center.y, b.getContourArea(), b.getDensity(), b.getAspectRatio()));
        }
        for (ColorBlobLocatorProcessor.Blob b : yellowBlobs) {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format(Locale.US,
                    "Yellow: (%3d,%3d) %5d  %4.2f   %5.2f",
                    (int) boxFit.center.x, (int) boxFit.center.y, b.getContourArea(), b.getDensity(), b.getAspectRatio()));
        }
    }
}
