package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ZeboltsTeleOpDecode extends LinearOpMode {
    //DEFINING MOTORS ***
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor topshooter; //Shooting Motor
    public DcMotor bottomshooter; //Shooting Motor
    public DcMotor intake; //The intake
    public Servo transfer; //The motors in the chamber
    public DcMotor turretMotor; //The turret rotator
    public Servo hood;


    @Override
    public void runOpMode() throws InterruptedException {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        topshooter = hardwareMap.get(DcMotor.class, "shooter 2");
        turretMotor = hardwareMap.get(DcMotor.class, "turret ring");
        hood = hardwareMap.get(Servo.class, "angle changer");
        transfer = hardwareMap.get(Servo.class, "transfer");


        //DEFINING MOTOR DIRECTION***
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()) {
            //DRIVING ***
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double turret = gamepad2.left_stick_x;
            double pythogoreanDistance = 0;
            double angleWebcam = 0;

            if (gamepad1.right_bumper) {
                frontleft.setPower(-(drive - turn - strafe) * 0.5); //Pos for strafe
                frontright.setPower((-drive - turn - strafe) * 0.5); //Neg for strafe
                backleft.setPower(-(drive - turn + strafe) * 0.5); //Neg for strafe
                backright.setPower(-(drive + turn - strafe) * 0.5); //Pos for strafe
            } else {
                frontleft.setPower(-(drive - turn - strafe)); //Pos for strafe
                frontright.setPower((-drive - turn - strafe)); //Neg for strafe
                backleft.setPower(-(drive - turn + strafe)); //Neg for strafe
                backright.setPower(-(drive + turn - strafe)); //Pos for strafe
            }


            //ACTUAL BALL LAUNCHER
            if (gamepad1.dpad_up) {
                bottomshooter.setPower(-0.51);//Original Power
                topshooter.setPower(0.51);//Original Power
                hood.setPosition(1);
                intake.setPower(-0.8);
            } else if (gamepad1.dpad_right) {
                bottomshooter.setPower(-0.56); //Original Power
                topshooter.setPower(0.56);//Original Power
                hood.setPosition(0.75);
                intake.setPower(-0.5);
            } else if (gamepad1.dpad_down) {
                bottomshooter.setPower(-0.78); //Original Power
                topshooter.setPower(0.78);//Original Power
                hood.setPosition(0.65);
                intake.setPower(-0.4);
            } else if (gamepad1.dpad_left) {
                bottomshooter.setPower(0); //Original Power
                topshooter.setPower(0);//Original Power
                hood.setPosition(1);
                intake.setPower(0);

            }


            if (gamepad1.right_trigger == 0) {
                transfer.setPosition(1);
            } else
                transfer.setPosition(0.85);

            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(-1);
            } else
                intake.setPower(0);

            if (gamepad2.left_trigger > 0.1) {
                intake.setPower(1);
            } else
                intake.setPower(0);


        }
    }
}

