package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanums.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous
public class AprilTagWebcamExample extends OpMode {
    private double posX;
    private double posY;
    private double posZ;
    private double bearingAngle;
    private double distanceDistance;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
    }


    @Override
    public void loop(){
        //update the vision portal
        aprilTagWebcam.update();
        //AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        //aprilTagWebcam.displayDetectionTelemetry(id20);


        //posX = aprilTagWebcam.getPositionX(id20);
        //posY = aprilTagWebcam.getPositionY(id20);
        //posZ = aprilTagWebcam.getPositionZ(id20);


        //Detecting Ball Order
        if (aprilTagWebcam.getTagBySpecificId(23) != null){
            telemetry.addLine("Purple Purple Green");
            AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);
            aprilTagWebcam.displayDetectionTelemetry(id23);
        } else if (aprilTagWebcam.getTagBySpecificId(22) != null){
            telemetry.addLine("Purple Green Purple");
            AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);
            aprilTagWebcam.displayDetectionTelemetry(id22);
        } else if (aprilTagWebcam.getTagBySpecificId(21) != null){
            telemetry.addLine("Green Purple Purple");
            AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);
            aprilTagWebcam.displayDetectionTelemetry(id21);
        } else if (aprilTagWebcam.getTagBySpecificId(20) != null){
            telemetry.addLine("Blue Base");
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            aprilTagWebcam.displayDetectionTelemetry(id20);

            bearingAngle = aprilTagWebcam.getPythagorean(id20);
            distanceDistance = aprilTagWebcam.getAngle(id20);

            if (bearingAngle > 2){
                telemetry.addLine("Turn right");
            } else if(bearingAngle < -2){
                telemetry.addLine("Turn left");
            } else {
                telemetry.addLine("Facing the right way!");
                telemetry.addData("Distance: ", distanceDistance);
            }
        } else if (aprilTagWebcam.getTagBySpecificId(24) != null){
            telemetry.addLine("Red Base");
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
            aprilTagWebcam.displayDetectionTelemetry(id24);

            bearingAngle = aprilTagWebcam.getPythagorean(id24);
            distanceDistance = aprilTagWebcam.getAngle(id24);

            if (bearingAngle > 2){
                telemetry.addLine("Turn right");
            } else if(bearingAngle < -2){
                telemetry.addLine("Turn left");
            } else {
                telemetry.addLine("Facing the right way!");
                telemetry.addData("Distance: ", distanceDistance);
            }

        }
    }
}
