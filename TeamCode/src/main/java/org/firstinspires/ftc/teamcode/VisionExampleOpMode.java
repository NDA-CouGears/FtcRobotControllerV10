package org.firstinspires.ftc.teamcode;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;

@Autonomous( name = "Vision", group = "Examples")
public class VisionExampleOpMode extends AutoMode {
    private WebcamName webcam1, webcam2;
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
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Elevated camera angled downward, good for sample location and identification
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        portal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(blueLocator, redLocator, yellowLocator, tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Default to looking from the elevated camera for sample search
        portal.setActiveCamera(webcam2);
    }

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        //waitForStart();

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Turn stream on or off
            if (gamepad1.y && (portal.getCameraState() == VisionPortal.CameraState.STREAMING)) {
                portal.stopStreaming();
            } else if (gamepad1.x && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                portal.resumeStreaming();
            }

            // Don't bother looking if the portal is not processing video
            if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                continue;
            }

            // Switch between cameras if requested
            if (gamepad1.a && (portal.getActiveCamera() == webcam2)) {
                portal.setActiveCamera(webcam1);
            } else if (gamepad1.b && (portal.getActiveCamera() == webcam1)) {
                portal.setActiveCamera(webcam2);
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

            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}
