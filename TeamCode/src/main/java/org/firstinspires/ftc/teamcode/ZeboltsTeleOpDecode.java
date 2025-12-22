package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanums.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp
public class ZeboltsTeleOpDecode extends LinearOpMode {
    //DEFINING MOTORS ***
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor topshooter; //Shooting Motor
    public DcMotor bottomshooter; //Shooting Motor
    public Servo trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor transferSystem; //The motors in the chamber
    public DcMotor turretRing; //The turret rotator
    public Servo hood;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void runOpMode() throws InterruptedException {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class,"front left");
        frontright = hardwareMap.get(DcMotor.class,"front right");
        backleft = hardwareMap.get(DcMotor.class,"back left");
        backright = hardwareMap.get(DcMotor.class,"back right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        topshooter = hardwareMap.get(DcMotor.class, "shooter 2");
        turretRing = hardwareMap.get(DcMotor.class, "turret ring");
        hood = hardwareMap.get(Servo.class, "angle changer");

        aprilTagWebcam.init(hardwareMap, telemetry);

        //DEFINING MOTOR DIRECTION***
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRing.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()){
            //DRIVING ***
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double turret = gamepad2.left_stick_x;
            double pythogoreanDistance = 0;
            double angleWebcam = 0;

            if (gamepad1.right_bumper) {
                frontleft.setPower(-(drive-turn-strafe) * 0.5); //Pos for strafe
                frontright.setPower((-drive-turn-strafe) * 0.5); //Neg for strafe
                backleft.setPower(-(drive-turn+strafe) * 0.5); //Neg for strafe
                backright.setPower(-(drive+turn-strafe) * 0.5); //Pos for strafe
            }
            else {
                frontleft.setPower(-(drive-turn-strafe)); //Pos for strafe
                frontright.setPower((-drive-turn-strafe)); //Neg for strafe
                backleft.setPower(-(drive-turn+strafe)); //Neg for strafe
                backright.setPower(-(drive+turn-strafe)); //Pos for strafe
            }
            turretRing.setPower(turret * 0.5);


            //ACTUAL BALL LAUNCHER
            if(gamepad1.dpad_up){
                bottomshooter.setPower(-0.51);//Original Power
                topshooter.setPower(0.51);//Original Power
                hood.setPosition(1);
            } else if(gamepad1.dpad_right){
                bottomshooter.setPower(-0.6); //Original Power
                topshooter.setPower(0.6);//Original Power
                hood.setPosition(0.8);
            } else if(gamepad1.dpad_down) {
                bottomshooter.setPower(-0.94); //Original Power
                topshooter.setPower(0.94);//Original Power
                hood.setPosition(0.65);
            }
              else if(gamepad1.dpad_left) {
                    bottomshooter.setPower(0); //Original Power
                    topshooter.setPower(0);//Original Power
                    hood.setPosition(1);

            }


            //INTAKE***

            /* if (gamepad1.dpad_up) {
                hood.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                hood.setPosition(0.28);
            }
            if (gamepad1.dpad_down) {
                hood.setPosition(0.4);
            }*/

            if(gamepad2.right_trigger > 0.1){
                intake.setPower(-1);
            } else
                intake.setPower(0);

            if(gamepad2.left_trigger > 0.1){
                intake.setPower(1);
            } else
                intake.setPower(0);
            }

            //Turret and Webcam MAGIC
            aprilTagWebcam.update();

            //IF THE ANGLE IS LESS THAN -4, TURN THE TURRET TO THE LEFT.
            //IF THE ANGLE IS GREATER THAN 4, TURN THE TURRET TO THE RIGHT

            if (aprilTagWebcam.getTagBySpecificId(20) != null){
                AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
                aprilTagWebcam.displayDetectionTelemetry(id20);

                //pythogoreanDistance = aprilTagWebcam.getPythagorean(id20);
                //angleWebcam = aprilTagWebcam.getAngle(id20);


                telemetry.addData("Bearing Value", aprilTagWebcam.getAngle(id20));
                //aprilTagWebcam.displayDetectionTelemetry(id20);

                //if (angleWebcam > 0.1){
                   // telemetry.addLine("Positive Bearing");
                //} else if(angleWebcam < -0.1){
                  //  telemetry.addLine("Negative Bearing");
                //} else {
                  //  turretRing.setPower(0);
                //}
            } else {
                telemetry.addLine("QR Code not found.");
            }


           /* if(gamepad2.left_trigger > 0.01){
               intake.setPower(1);
               transferSystem.setPower(1);


           } else if (gamepad2.right_trigger > 0.01){
               intake.setPower(-1);
               transferSystem.setPower(-1);
           } else{
               intake.setPower(0);
               transferSystem.setPower(0);*/
        }
    }



