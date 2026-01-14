package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Zebolts TeleOp Fast AprilTag", group = "TeleOp")
public class ZeboltsTeleOpDecode extends LinearOpMode {

    // DRIVE MOTORS
    public DcMotor frontleft;
    public DcMotor frontright;
    public DcMotor backleft;
    public DcMotor backright;

    // SHOOTER MOTORS
    public DcMotor bottomshooter;
    public DcMotor intake;

    // SERVOS
    public Servo transfer;
    public Servo hood;

    // TURRET
    public DcMotor turretMotor;

    // APRILTAG VISION
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // AprilTag tracking state
    private boolean aprilTagTrackingEnabled = false;
    private boolean lastGamepad2A = false;

    // Detection smoothing for fast movement
    private double lastValidBearing = 0;
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT_MS = 500;

    // Turret configuration constants
    private static final double TURRET_MANUAL_POWER = 0.5;
    private static final double TURRET_ALIGN_POWER = 0.25;
    private static final double BEARING_TOLERANCE = 3.0;
    private static final int TARGET_TAG_ID = -1; // -1 for any tag, or specific ID

    // Camera settings for fast motion
    private static final int CAMERA_EXPOSURE_MS = 6;  // Fast exposure to reduce motion blur
    private static final int CAMERA_GAIN = 220;        // High gain to compensate for low exposure

    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZE HARDWARE
        initDriveMotors();
        initShooterSystem();
        initTurret();
        initAprilTag();

        telemetry.addData("Status", "Initialized - Optimized for Fast Turning");
        telemetry.addData(">", "Press A on gamepad2 to toggle AprilTag tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // TOGGLE APRILTAG TRACKING (Gamepad2 A button)
            boolean currentGamepad2A = gamepad2.a;
            if (currentGamepad2A && !lastGamepad2A) {
                aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
                if (!aprilTagTrackingEnabled) {
                    turretMotor.setPower(0); // Stop turret when disabling tracking
                }
            }
            lastGamepad2A = currentGamepad2A;

            // DRIVING
            handleDriving();

            // SHOOTER CONTROLS
            handleShooter();

            // TRANSFER CONTROLS
            handleTransfer();

            // INTAKE CONTROLS
            handleIntake();

            // TURRET CONTROLS
            handleTurret();

            // TELEMETRY
            displayTelemetry();
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize drive motors
     */
    private void initDriveMotors() {
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Initialize shooter system
     */
    private void initShooterSystem() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        hood = hardwareMap.get(Servo.class, "angle changer");
        transfer = hardwareMap.get(Servo.class, "transfer");
    }

    /**
     * Initialize turret motor
     */
    private void initTurret() {
        turretMotor = hardwareMap.get(DcMotor.class, "turret ring");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize AprilTag detection - OPTIMIZED FOR FAST TURNING
     */
    private void initAprilTag() {
        // Build AprilTag processor with optimized settings
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)  // Disable drawing for faster processing
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)  // More robust detection
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Build vision portal with optimized settings
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))  // Lower resolution = faster processing
                .enableLiveView(false)  // Disable live view for better performance
                .setAutoStopLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Wait for camera to start streaming
        telemetry.addData("Camera", "Initializing...");
        telemetry.update();

        while (opModeInInit() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
            telemetry.addData("Camera", "Waiting... %s", visionPortal.getCameraState());
            telemetry.update();
        }

        // Configure camera for fast motion detection
        configureCameraForFastMotion();

        telemetry.addData("Camera", "Ready!");
        telemetry.update();
    }

    /**
     * Configure camera settings to reduce motion blur during fast turning
     */
    private void configureCameraForFastMotion() {
        try {
            // Set manual exposure for fast shutter speed (reduces motion blur)
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
                telemetry.addData("Exposure", "Set to %d ms", CAMERA_EXPOSURE_MS);
            } else {
                telemetry.addData("Exposure", "Not available");
            }

            // Increase gain to compensate for fast exposure (brightens image)
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(CAMERA_GAIN);
                telemetry.addData("Gain", "Set to %d", CAMERA_GAIN);
            } else {
                telemetry.addData("Gain", "Not available");
            }

            telemetry.addData("Camera Config", "Optimized for fast motion");
            telemetry.update();
            sleep(1000);

        } catch (Exception e) {
            telemetry.addData("Camera Config Error", e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Handle driving controls
     */
    private void handleDriving() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        double speedMultiplier = gamepad1.right_bumper ? 0.5 : 1.0;

        frontleft.setPower(-(drive - turn - strafe) * speedMultiplier);
        frontright.setPower((-drive - turn - strafe) * speedMultiplier);
        backleft.setPower(-(drive - turn + strafe) * speedMultiplier);
        backright.setPower(-(drive + turn - strafe) * speedMultiplier);
    }

    /**
     * Handle shooter controls (gamepad1 D-pad)
     */
    private void handleShooter() {
        if (gamepad1.dpad_up) {
            // Close range shot
            bottomshooter.setPower(-0.6);
            hood.setPosition(0.9);
            intake.setPower(-0.8);
        } else if (gamepad1.dpad_right) {
            // Medium range shot
            bottomshooter.setPower(-0.78);
            hood.setPosition(0.75);
            intake.setPower(-0.5);
        } else if (gamepad1.dpad_down) {
            // Long range shot
            bottomshooter.setPower(-0.95);
            hood.setPosition(0.65);
            intake.setPower(-0.4);
        } else if (gamepad1.dpad_left) {
            // Stop shooter
            bottomshooter.setPower(0);
            hood.setPosition(1);
            intake.setPower(0);
        }
    }

    /**
     * Handle transfer servo (gamepad1 right trigger)
     */
    private void handleTransfer() {
        if (gamepad1.right_trigger == 0) {
            transfer.setPosition(0.85);
        } else {
            transfer.setPosition(1);
        }
    }

    /**
     * Handle intake controls (gamepad2 triggers)
     */
    private void handleIntake() {
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(-1);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    /**
     * Handle turret controls - either manual or AprilTag tracking
     */
    private void handleTurret() {
        if (aprilTagTrackingEnabled) {
            // APRILTAG TRACKING MODE - with smoothing for fast turns
            trackAprilTagWithSmoothing();
        } else {
            // MANUAL CONTROL MODE (gamepad2 left stick X)
            double turret = gamepad2.left_stick_x;
            turretMotor.setPower(turret * TURRET_MANUAL_POWER);
        }
    }

    /**
     * Track AprilTag with smoothing to handle fast turning
     */
    private void trackAprilTagWithSmoothing() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        long currentTime = System.currentTimeMillis();

        if (detections.size() == 0) {
            // No tag detected - use last known bearing if recent
            if (currentTime - lastDetectionTime < DETECTION_TIMEOUT_MS) {
                // Continue tracking with last known position at reduced power
                double adjustPower = lastValidBearing / 18.0;
                adjustPower = Math.max(-TURRET_ALIGN_POWER, Math.min(TURRET_ALIGN_POWER, adjustPower));
                turretMotor.setPower(adjustPower * 0.5);  // 50% power when using old data
                telemetry.addData("Tracking Mode", "Using last known position");
            } else {
                turretMotor.setPower(0);
                telemetry.addData("Tracking Mode", "No tag detected");
            }
            return;
        }

        // Find target tag
        AprilTagDetection targetDetection = null;
        if (TARGET_TAG_ID == -1) {
            // Use first detected tag
            targetDetection = detections.get(0);
        } else {
            // Look for specific tag ID
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_TAG_ID) {
                    targetDetection = detection;
                    break;
                }
            }
        }

        if (targetDetection == null) {
            turretMotor.setPower(0);
            return;
        }

        // Update tracking data
        double bearing = targetDetection.ftcPose.bearing;
        lastValidBearing = bearing;
        lastDetectionTime = currentTime;

        // Display detection info
        telemetry.addData("Range from target", "%.1f inches", targetDetection.ftcPose.range);
        telemetry.addData("Bearing", "%.1f degrees", bearing);
        telemetry.addData("Tracking Mode", "Active detection");

        // Check hard stops
        int currentPosition = turretMotor.getCurrentPosition();
        if (currentPosition > 658) {
            telemetry.addLine("WARNING: Turret at positive limit!");
            turretMotor.setPower(0);
            return;
        } else if (currentPosition < -852) {
            telemetry.addLine("WARNING: Turret at negative limit!");
            turretMotor.setPower(0);
            return;
        }

        // Check if centered
        if (Math.abs(bearing) < BEARING_TOLERANCE) {
            turretMotor.setPower(0);
            telemetry.addData("Status", "ON TARGET");
            return;
        }

        // Proportional control for smooth tracking
        double adjustPower = bearing / 18.0;
        adjustPower = Math.max(-TURRET_ALIGN_POWER, Math.min(TURRET_ALIGN_POWER, adjustPower));

        turretMotor.setPower(adjustPower);
        telemetry.addData("Status", "Tracking...");
    }

    /**
     * Display telemetry information
     */
    private void displayTelemetry() {
        telemetry.addData("AprilTag Tracking", aprilTagTrackingEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Control Mode", aprilTagTrackingEnabled ? "Auto" : "Manual");

        if (aprilTagTrackingEnabled) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Tags Detected", detections.size());

            if (detections.size() > 0) {
                AprilTagDetection detection = detections.get(0);
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
                telemetry.addData("Range", "%.2f inches", detection.ftcPose.range);
            }

            // Show time since last detection
            long timeSinceDetection = System.currentTimeMillis() - lastDetectionTime;
            telemetry.addData("Last Detection", "%d ms ago", timeSinceDetection);
        }

        telemetry.addData("Turret Position", turretMotor.getCurrentPosition());
        telemetry.addData("", "");
        telemetry.addData("Controls", "Gamepad2 A = Toggle AprilTag");

        telemetry.update();
    }
}