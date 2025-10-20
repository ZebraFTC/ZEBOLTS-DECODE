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
    public DcMotor shooter; //Shooting Motor
    public Servo trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor transferSystem; //The motors in the chamber

    @Override
    public void runOpMode() throws InterruptedException {
            //DEFINING HARDWARE MAP ***
            frontleft = hardwareMap.get(DcMotor.class,"front left");
            frontright = hardwareMap.get(DcMotor.class,"front right");
            backleft = hardwareMap.get(DcMotor.class,"back left");
            backright = hardwareMap.get(DcMotor.class,"back right");
            trigger = hardwareMap.get(Servo.class,"shooter left");
            intake = hardwareMap.get(DcMotor.class, "intake");
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            transferSystem = hardwareMap.get(DcMotor.class,"transfer");

        //DEFINING MOTOR DIRECTION***
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()){
            //DRIVING ***
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            frontleft.setPower(-(drive-turn-strafe)); //Pos for strafe
            frontright.setPower((-drive-turn-strafe)); //Neg for strafe
            backleft.setPower(-(drive-turn+strafe)); //Neg for strafe
            backright.setPower(-(drive+turn-strafe)); //Pos for strafe

            //TRIGGER INTO BALL LAUNCHER ***
            if (gamepad1.a) {
                trigger.setPosition(0.2); //IDK VALUE (Launch Position)
            }else{
                trigger.setPosition(0); //IDK VALUE (Original Position)
            }

            //ACTUAL BALL LAUNCHER
            if(gamepad2.dpad_up){
                shooter.setPower(0.6); //Original Power
            } else if(gamepad2.dpad_left){
                shooter.setPower(0.8); //Sniping Power
            } else if(gamepad2.dpad_right){
                shooter.setPower(0.66); //Sniping Power
            } else if(gamepad2.dpad_down){
                shooter.setPower(0); //Stopping Power
            }

            //INTAKE***

            if(gamepad2.right_trigger > 0.1){
                intake.setPower(-1);
                transferSystem.setPower(-1);
            } else if(gamepad2.right_bumper){
                intake.setPower(0);
                transferSystem.setPower(0);
            }

            if(gamepad2.left_trigger > 0.1){
                intake.setPower(1);
                transferSystem.setPower(1);
            } else if(gamepad2.left_bumper){
                intake.setPower(0);
                transferSystem.setPower(0);
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
    }

