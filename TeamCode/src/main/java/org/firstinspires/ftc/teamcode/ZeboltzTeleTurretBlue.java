package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanums.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp
public class ZeboltzTeleTurretBlue extends LinearOpMode{
    //DEFINING MOTORS ***
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor topshooter; //Shooting Motor
    public DcMotor bottomshooter; //Shooting Motor
    public Servo trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor turretRing; //The turret rotator
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void runOpMode() throws InterruptedException {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class,"front left");
        frontright = hardwareMap.get(DcMotor.class,"front right");
        backleft = hardwareMap.get(DcMotor.class,"back left");
        backright = hardwareMap.get(DcMotor.class,"back right");
        trigger = hardwareMap.get(Servo.class,"transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        turretRing = hardwareMap.get(DcMotor.class, "turret ring");

        aprilTagWebcam.init(hardwareMap, telemetry);



        //DEFINING MOTOR DIRECTION***
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()){
            //DRIVING ***
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
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

            //TRIGGER INTO BALL LAUNCHER ***
            if (gamepad1.a) {
                trigger.setPosition(0); //IDK VALUE (Launch Position)
            } else if (gamepad1.b) {
                trigger.setPosition(0); //IDK VALUE (Launch Position)
            } else {
                trigger.setPosition(0);
            }


            //ACTUAL BALL LAUNCHER
            if(gamepad2.dpad_up){
                topshooter.setPower(-0.59); //Original Power
                bottomshooter.setPower(-0.59); //Original Power
            } else if(gamepad2.dpad_left){
                topshooter.setPower(-0.95); //Sniping Power
                bottomshooter.setPower(-0.95); //Original Power
            } else if(gamepad2.dpad_right){
                topshooter.setPower(-0.51); //Sniping Power
                bottomshooter.setPower(-0.51); //Original Power
            } else if(gamepad2.dpad_down    ) {
                topshooter.setPower(0); //Stopping Power
                bottomshooter.setPower(0); //Original Power
            }    else if(gamepad2.a){
                topshooter.setPower(0.2); //Stopping Power
                bottomshooter.setPower(0.2); //Original Power
            }





            //Turret and Webcam MAGIC
            aprilTagWebcam.update();

            //IF THE ANGLE IS LESS THAN -4, TURN THE TURRET TO THE LEFT.
            //IF THE ANGLE IS GREATER THAN 4, TURN THE TURRET TO THE RIGHT

            if (aprilTagWebcam.getTagBySpecificId(20) != null){
                AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
                pythogoreanDistance = aprilTagWebcam.getPythagorean(id20);
                angleWebcam = aprilTagWebcam.getAngle(id20);

                if (angleWebcam > 4){
                    telemetry.addLine("Positive Bearing");
                } else if(angleWebcam < -4){
                    telemetry.addLine("Negative Bearing");
                } else {
                    turretRing.setPower(0);
                }
            }

            if (gamepad2.left_bumper){

            }



            //INTAKE***


            if(gamepad2.right_trigger > 0.1){
                intake.setPower(-1);
            } else if(gamepad2.right_bumper){
                intake.setPower(0);
            }

            if(gamepad2.left_trigger > 0.1){
                intake.setPower(1);
            }
        }
    }
}

