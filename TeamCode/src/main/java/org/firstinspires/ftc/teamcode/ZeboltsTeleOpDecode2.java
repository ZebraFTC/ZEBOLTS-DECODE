package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Zebolts TeleOp with AprilTag", group = "TeleOp")
public class ZeboltsTeleOpDecode2 extends LinearOpMode {

    // DRIVE MOTORS
    public DcMotor frontleft;
    public DcMotor frontright;
    public DcMotor backleft;
    public DcMotor backright;

    // SHOOTER MOTORS
    public DcMotor topshooter;
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

    // Turret configuration constants
    private static final double TURRET_MANUAL_POWER = 0.5;
    private static final double TURRET_ALIGN_POWER = 0.25;
    private static final double BEARING_TOLERANCE = 3.0;
    private static final int TARGET_TAG_ID = -1; // -1 for any tag, or specific ID

    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZE HARDWARE
        initDriveMotors();
        initShooterSystem();
        initTurret();
        initAprilTag();

        telemetry.addData("Status", "Initialized");
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
        topshooter = hardwareMap.get(DcMotor.class, "shooter 2");
        hood = hardwareMap.get(Servo.class, "angle changer");
        transfer = hardwareMap.get(Servo.class, "transfer");
    }

    /**
     * Initialize turret motor
     */
    private void initTurret() {
        turretMotor = hardwareMap.get(DcMotor.class, "turret ring");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
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
            bottomshooter.setPower(-0.51);
            topshooter.setPower(0.51);
            hood.setPosition(1);
            intake.setPower(-0.8);
        } else if (gamepad1.dpad_right) {
            // Medium range shot
            bottomshooter.setPower(-0.56);
            topshooter.setPower(0.56);
            hood.setPosition(0.75);
            intake.setPower(-0.5);
        } else if (gamepad1.dpad_down) {
            // Long range shot
            bottomshooter.setPower(-0.78);
            topshooter.setPower(0.78);
            hood.setPosition(0.65);
            intake.setPower(-0.4);
        } else if (gamepad1.dpad_left) {
            // Stop shooter
            bottomshooter.setPower(0);
            topshooter.setPower(0);
            hood.setPosition(1);
            intake.setPower(0);
        }
    }

    /**
     * Handle transfer servo (gamepad1 right trigger)
     */
    private void handleTransfer() {
        if (gamepad1.right_trigger == 0) {
            transfer.setPosition(1);
        } else {
            transfer.setPosition(0.85);
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
            // APRILTAG TRACKING MODE
            trackAprilTag();
        } else {
            // MANUAL CONTROL MODE (gamepad2 left stick X)
            double turret = gamepad2.left_stick_x;
            turretMotor.setPower(turret * TURRET_MANUAL_POWER);
        }
    }

    /**
     * Track AprilTag and adjust turret to keep it centered
     */
    private void trackAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.size() == 0) {
            // No tag detected - stop turret
            turretMotor.setPower(0);
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

        double bearing = targetDetection.ftcPose.bearing;

        // Check if centered
        if (Math.abs(bearing) < BEARING_TOLERANCE) {
            turretMotor.setPower(0);
            return;
        }

        // Proportional control for smooth tracking
        double adjustPower = bearing / 18.0;
        adjustPower = Math.max(-TURRET_ALIGN_POWER, Math.min(TURRET_ALIGN_POWER, adjustPower));

        turretMotor.setPower(adjustPower);
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
        }

        telemetry.addData("Turret Position", turretMotor.getCurrentPosition());
        telemetry.addData("", "");
        telemetry.addData("Controls", "Gamepad2 A = Toggle AprilTag");
        telemetry.update();
    }
}