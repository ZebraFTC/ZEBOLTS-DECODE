package org.firstinspires.ftc.teamcode.mechanums;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection tag) {
        if (tag == null) {
            telemetry.addLine("No tag detected");
            return;
        }

        telemetry.addLine("\n==== Tag ID: " + tag.id);

        // Metadata may be null depending on tag family
        if (tag.metadata != null) {
            telemetry.addLine("Tag Name: " + tag.metadata.name);
        } else {
            telemetry.addLine("Tag Name: UNKNOWN");
        }

        // Check for null ftcPose (prevents runtime crashes)
        if (tag.ftcPose != null) {
            telemetry.addLine(String.format("XYZ (cm): %.1f  %.1f  %.1f",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

            telemetry.addLine(String.format("PRY (deg): %.1f  %.1f  %.1f",
                    tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));

            telemetry.addLine(String.format("RBE: %.1f (cm)  %.1f (deg)  %.1f (deg)",
                    tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
        } else {
            telemetry.addLine("ftcPose is NULL — cannot compute position!");
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) return detection;
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    // Safe wrappers that avoid null ftcPose crashes
    public double getPositionX(AprilTagDetection tag) {
        return (tag != null && tag.ftcPose != null) ? tag.ftcPose.x : 0.0;
    }

    public double getPositionY(AprilTagDetection tag) {
        return (tag != null && tag.ftcPose != null) ? tag.ftcPose.y : 0.0;
    }

    public double getPositionZ(AprilTagDetection tag) {
        return (tag != null && tag.ftcPose != null) ? tag.ftcPose.z : 0.0;
    }

    public double getPositionAngle(AprilTagDetection tag) {
        return (tag != null && tag.ftcPose != null) ? tag.ftcPose.yaw : 0.0;
    }
}
