package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ZeboltzTeleTurret extends LinearOpMode {
    //DEFINING MOTORS ***
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor topshooter; //Shooting Motor
    public DcMotor bottomshooter; //Shooting Motor
    public DcMotor trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor transferSystem; //The motors in the chamber


    @Override
    public void runOpMode() throws InterruptedException {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class,"front left");
        frontright = hardwareMap.get(DcMotor.class,"front right");
        backleft = hardwareMap.get(DcMotor.class,"back left");
        backright = hardwareMap.get(DcMotor.class,"back right");
        trigger = hardwareMap.get(DcMotor.class,"transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        topshooter = hardwareMap.get(DcMotor.class, "shooter 2");


        //DEFINING MOTOR DIRECTION***
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()){

        }
    }
}