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
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init(){

    }

    @Override
    public void loop(){
        //update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        posX = aprilTagWebcam.getPositionX(id20);
        posY = aprilTagWebcam.getPositionY(id20);
        posZ = aprilTagWebcam.getPositionZ(id20);

        //Detecting Ball Order
        if (aprilTagWebcam.getTagBySpecificId(23) != null){
            telemetry.addLine("Purple Purple Green");
        } else if (aprilTagWebcam.getTagBySpecificId(22) != null){
            telemetry.addLine("Purple Green Purple");
        } else if (aprilTagWebcam.getTagBySpecificId(21) != null){
            telemetry.addLine("Green Purple Purple");
        }
    }
}
