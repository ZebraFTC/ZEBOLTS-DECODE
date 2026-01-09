package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Turret Scanner", group = "Vision")
public class AprilTagBetter extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor turretMotor;

    // Turret configuration constants
    private static final double TURRET_SCAN_POWER = 0.6;  // Slow scan speed
    private static final double TURRET_ALIGN_POWER = 0.25; // Slow alignment speed
    private static final int TURRET_SCAN_TIMEOUT = 10000;  // 10 seconds
    private static final double BEARING_TOLERANCE = 3.0;   // Degrees - how centered is "good enough"

    // **CHANGE THIS TO SET WHICH APRILTAG TO FIND**
    // Set to -1 to find ANY tag, or set to a specific ID (1, 2, 3, etc.)
    private static final int TARGET_TAG_ID = -1;  // Change this number!

    // For bidirectional scan: how far to scan in each direction (encoder ticks)
    private static final int SCAN_RANGE_TICKS = 634;  // Adjust for your turret

    @Override
    public void runOpMode() {
        initAprilTag();
        initTurret();

        // Wait for the DS start button to be touched
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Scan for AprilTag by rotating turret (bidirectional scan)
            boolean tagFound;

            if (TARGET_TAG_ID == -1) {
                // Scan for ANY AprilTag
                telemetry.addData("Target", "Any AprilTag");
                telemetry.update();
                tagFound = bidirectionalScan();
            } else {
                // Scan for SPECIFIC AprilTag ID
                telemetry.addData("Target", "AprilTag ID " + TARGET_TAG_ID);
                telemetry.update();
                tagFound = bidirectionalScanForSpecificTag(TARGET_TAG_ID);
            }

            if (tagFound) {
                // Center the turret on the detected tag
                centerTurretOnTag();

                // Track the AprilTag continuously
                trackAprilTag();
            } else {
                telemetry.addData("Status", "No AprilTag found during scan");
                telemetry.update();
            }
        }

        // Save CPU resources when done
        stopTurret();
        visionPortal.close();
    }

    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create the vision portal using the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Initialize the turret motor
     */
    private void initTurret() {
        // Get the turret motor from hardware map
        // Make sure "turretMotor" matches the name in your robot configuration
        turretMotor = hardwareMap.get(DcMotor.class, "turret ring");

        // Reset encoder and set mode
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor to brake when power is zero (important for turret stability)
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Scan by rotating turret left to right until AprilTag is found
     * Returns true if tag found, false if timeout
     */
    private boolean scanForAprilTag() {
        telemetry.addData("Status", "Scanning for AprilTag...");
        telemetry.update();

        long startTime = System.currentTimeMillis();

        // Start rotating turret clockwise
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive()) {
            // Check for timeout
            if (System.currentTimeMillis() - startTime > TURRET_SCAN_TIMEOUT) {
                stopTurret();
                telemetry.addData("Status", "Scan timeout - no tag found");
                telemetry.update();
                return false;
            }

            // Check for AprilTag
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.size() > 0) {
                stopTurret();
                telemetry.addData("Status", "AprilTag Found!");
                telemetry.addData("Tag ID", detections.get(0).id);
                telemetry.update();
                sleep(500);
                return true;
            }

            // Display scan status
            telemetry.addData("Status", "Scanning...");
            telemetry.addData("Turret Position", turretMotor.getCurrentPosition());
            telemetry.addData("Scan Time", (System.currentTimeMillis() - startTime) / 1000.0 + "s");
            telemetry.update();

            sleep(50);
        }

        return false;
    }

    /**
     * Scan for a specific AprilTag ID
     */
    private boolean scanForSpecificTag(int targetTagId) {
        telemetry.addData("Status", "Scanning for Tag ID: " + targetTagId);
        telemetry.update();

        long startTime = System.currentTimeMillis();
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive()) {
            if (System.currentTimeMillis() - startTime > TURRET_SCAN_TIMEOUT) {
                stopTurret();
                return false;
            }

            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    stopTurret();
                    telemetry.addData("Status", "Target Tag Found!");
                    telemetry.addData("Tag ID", targetTagId);
                    telemetry.update();
                    sleep(500);
                    return true;
                }
            }

            telemetry.addData("Status", "Scanning for Tag " + targetTagId);
            telemetry.addData("Tags Visible", detections.size());
            telemetry.update();
            sleep(50);
        }

        return false;
    }

    /**
     * Center the turret on the detected AprilTag using bearing
     */
    private void centerTurretOnTag() {
        telemetry.addData("Status", "Centering turret on tag...");
        telemetry.update();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() == 0) {
                // Lost the tag
                stopTurret();
                telemetry.addData("Status", "Lost AprilTag");
                telemetry.update();
                break;
            }

            AprilTagDetection detection = detections.get(0);
            double bearing = detection.ftcPose.bearing;

            // Check if we're centered
            if (Math.abs(bearing) < BEARING_TOLERANCE) {
                stopTurret();
                telemetry.addData("Status", "Turret Centered!");
                telemetry.addData("Final Bearing", "%.2f degrees", bearing);
                telemetry.update();
                sleep(500);
                break;
            }

            // Adjust turret based on bearing
            // Positive bearing = tag is to the right, so rotate clockwise
            // Negative bearing = tag is to the left, so rotate counter-clockwise
            if (bearing > 0) {
                turretMotor.setPower(TURRET_ALIGN_POWER);
            } else {
                turretMotor.setPower(-TURRET_ALIGN_POWER);
            }

            telemetry.addData("Status", "Aligning...");
            telemetry.addData("Bearing", "%.2f degrees", bearing);
            telemetry.addData("Adjustment", bearing > 0 ? "Rotating Right" : "Rotating Left");
            telemetry.update();

            sleep(50);
        }
    }

    /**
     * Bidirectional scan - rotates one way, then reverses if no tag found
     */
    private boolean bidirectionalScan() {
        telemetry.addData("Status", "Starting bidirectional scan...");
        telemetry.update();

        int startPosition = turretMotor.getCurrentPosition();

        // Scan clockwise
        turretMotor.setTargetPosition(startPosition + SCAN_RANGE_TICKS);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.size() > 0) {
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                stopTurret();
                return true;
            }
            sleep(50);
        }

        // If not found, scan counter-clockwise
        turretMotor.setTargetPosition(startPosition - SCAN_RANGE_TICKS);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.size() > 0) {
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                stopTurret();
                return true;
            }
            sleep(50);
        }

        // Return to start position
        turretMotor.setTargetPosition(startPosition);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            sleep(50);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopTurret();
        return false;
    }

    /**
     * Bidirectional scan for a specific AprilTag ID
     */
    private boolean bidirectionalScanForSpecificTag(int targetTagId) {
        telemetry.addData("Status", "Scanning for Tag ID: " + targetTagId);
        telemetry.update();

        int startPosition = turretMotor.getCurrentPosition();

        // Scan clockwise
        turretMotor.setTargetPosition(startPosition + SCAN_RANGE_TICKS);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    stopTurret();
                    return true;
                }
            }
            sleep(50);
        }

        // Scan counter-clockwise
        turretMotor.setTargetPosition(startPosition - SCAN_RANGE_TICKS);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    stopTurret();
                    return true;
                }
            }
            sleep(50);
        }

        // Return to start
        turretMotor.setTargetPosition(startPosition);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_SCAN_POWER);

        while (opModeIsActive() && turretMotor.isBusy()) {
            sleep(50);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopTurret();
        return false;
    }

    /**
     * Continuously track and follow the AprilTag
     * Keeps the turret centered on the tag as it moves
     */
    private void trackAprilTag() {
        telemetry.addData("Status", "Tracking AprilTag...");
        telemetry.update();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() == 0) {
                // Lost the tag - stop and wait
                stopTurret();
                telemetry.addData("Status", "Tag lost - searching...");
                telemetry.update();
                sleep(100);
                continue;
            }

            AprilTagDetection detection = detections.get(0);
            double bearing = detection.ftcPose.bearing;

            // Proportional control - adjust speed based on how far off-center
            double adjustPower = bearing / 18.0;  // Divide by 30 to scale the response
            adjustPower = Math.max(-TURRET_ALIGN_POWER, Math.min(TURRET_ALIGN_POWER, adjustPower));

            // Apply power to follow the tag
            turretMotor.setPower(adjustPower);

            telemetry.addData("Status", "Tracking");
            telemetry.addData("Bearing", "%.2f degrees", bearing);
            telemetry.addData("Turret Power", "%.2f", adjustPower);
            telemetry.addData("Tag ID", detection.id);
            telemetry.update();

            sleep(20);
        }
    }

    /**
     * Stop the turret motor
     */
    private void stopTurret() {
        turretMotor.setPower(0);
    }

    /**
     * Display AprilTag detections to telemetry
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("Turret Position", turretMotor.getCurrentPosition());

        // Step through the list of detections and display info for each
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s",
                        detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}