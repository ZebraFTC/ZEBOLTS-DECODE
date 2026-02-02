//IMPORTING LIBRARIES
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class RedNearENCODER extends LinearOpMode {
    //DEFINING MOTORS
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotorEx topshooter; //Shooting Motor
    public DcMotor bottomshooter; //Shooting Motor
    public DcMotor intake; //The intake
    public Servo ballBooter; //The ball booter
    public DcMotor turretRing; //The turret rotator
    public Servo hood;
    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;
    public DcMotor turretring;
    private int turretPos;
    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        ballBooter = hardwareMap.get(Servo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        topshooter = hardwareMap.get(DcMotorEx.class, "shooter 2");
        hood = hardwareMap.get(Servo.class, "angle changer");
        turretring = hardwareMap.get(DcMotor.class, "turret ring");


        //RESTARTING ENCODERS
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretring.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);


        //SETTING NEW VARIABLES
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;
        turretPos = 0;



        //THE AUTO ITSELF
        waitForStart();

        //SECTION 1: SHOOTING PRELOADED BALLS
        shooter(0.54);
        turnTurret(0.5,-125);
        drive(-700,-700,-700,-700,0.4);
        drive(-900,900,900,-900,0.4);
        shoot(0.54,1.5,0,1,0.95);
        shoot(0.54,2,1,0.85,0.95);
        shooter(0);
        shoot(0,0.1,0,1,0.95);
        drive(35,35,-35,-35,0.25);
        drive(-800,800,800,-800,0.4);

        //SECTION 2: THE GATE OPENER
        intake(1);
        drive(1300,1300,1300,1300,0.25);
        drive(-900,-900,-900,-900,0.4);
        intake(0);
        //drive(1200,1200,1200,1200,0.25); gate in
        //drive(-1200,-1200,-1200,-1200,0.4); gate out
        drive(750,-750,-750,750,0.4);
        turnTurret(0.5,-140);
        shoot(0.49,1.5,0,1,0.95);
        shoot(0.49,2,1,0.85,0.95);
        shooter(0);
        shoot(0,0.1,0,1,0.95);
        drive(30,30,-30,-30,0.25);
        drive(-1500,1500,1500,-1500,0.4);

        //SECTION 2: THE GATE OPENER
        intake(1);
        drive(1250,1250,1250,1250,0.25);
        drive(-1200,-1200,-1200,-1200,0.4);
        intake(0);
        drive(1500,-1500,-1500,1500,0.4);
        shoot(0.48,1.5,0,1,0.95);
        shoot(0.48,2,1,0.85,0.95);
        shooter(0);
        shoot(0,0.1,0,1,0.95);
        drive(30,30,-30,-30,0.25);
        drive(-700,700,700,-700,0.4);







        /*
        //SECTION 4: MORE INTAKE AND SHOTS
        drive(-1350,1350,1350,-1350,0.4);
        intake(1);
        drive(1200,1200,1200,1200,0.25);
        drive(-800,-800,-800,-800,0.4);
        drive(-1350,1350,1350,-1350,0.4);
        intake(0);
        turnTurret(0.5,-100);
        shoot(0.9,1.5,0,1,0.95);
        shoot(0.9,2,1,0.85,0.95);
        shooter(0);
        shoot(0,0.1,0,1,0.95);
        */









       /*
       --TEMPLATES--
       drive(back left target,front left target,back right target,front right target,speed);        //Drive function
       drive(1000,1000,1000,1000,0.25);        //Drives forwards
       drive(-1000,-1000,-1000,-1000,0.25);        //Drives backwards
       drive(1000,1000,-1000,-1000,0.1);       //Turns right
       drive(-1000,-1000,1000,1000,0.1);       //Turns left
       drive(-1000,1000,1000,-1000,0.1);       //Strafes some direction

       shoot(shooter power, time (seconds), intake power, ball booter position, hood position, turretring power);            //Shoot function
       shoot(0, 5, 1, 4, 0.95, 0);            //Intakes/Transfers Balls for 5 seconds
       shoot(0.7, 5, 1, 0, 0.95, 0);               //Shoots 1 Ball for 5 seconds
       */
    }


    //DRIVE FUNCTION
    private void drive(int leftBackTarget, int leftFrontTarget, int rightBackTarget, int rightFrontTarget, double speed) {
        leftBackPos += leftBackTarget;
        leftFrontPos += leftFrontTarget;
        rightBackPos += rightBackTarget;
        rightFrontPos += rightFrontTarget;


        frontleft.setTargetPosition(leftFrontPos);
        backleft.setTargetPosition(leftBackPos);
        frontright.setTargetPosition(rightFrontPos);
        backright.setTargetPosition(rightBackPos);


        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);


        while (opModeIsActive() && frontleft.isBusy() && backleft.isBusy() && frontright.isBusy() && backright.isBusy()) {
            idle();
        }
    }

    //Shooting method
    private void shoot(double shooterPower, double time, double intakePower, double ballBooterPOS, double hoodPOS) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            intake.setPower(-1 * intakePower);
            bottomshooter.setPower((-1 * shooterPower));
            topshooter.setVelocity(-1 * 2800 * shooterPower);
            ballBooter.setPosition(ballBooterPOS);
            hood.setPosition(hoodPOS);
        }
    }

    private void turnTurret(double turretSpeed, int turretTarget){
        turretPos += turretTarget;
        turretring.setTargetPosition(turretPos);
        turretring.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretring.setPower(turretSpeed);

        while (opModeIsActive() && turretring.isBusy()){
            idle();
        }
    }
    private void shooter(double shooterSpeed){
        bottomshooter.setPower((-1 * shooterSpeed));
        topshooter.setVelocity(- 1 * 2800 * shooterSpeed);
    }
    private void intake(double intakeSpeed){
        intake.setPower(-1 * intakeSpeed);
    }
}
