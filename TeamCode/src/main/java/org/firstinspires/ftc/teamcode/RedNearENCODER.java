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


        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);


        //SETTING NEW VARIABLES
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;



        //THE AUTO ITSELF
        waitForStart(); //Hrithik we can run the shoot method to intake and not shoot if desired

        //Shooting Preloaded Balls
        drive(-888, -888, -888, -888, 0.5);//Drives backwards
        shoot(0.4, 3, 1, 1, 0.95, 0); //Shoots preloaded balls

        //Intaking 3 Balls
        drive(-400, -400, -400, -400, 0.5);//Drives backwards
        drive(900,900,-900,-900,0.4);//Turns right
        shoot(0, 5, 1, 0, 0.95, 0);     //Turns on Intake
        drive(1000,1000,1000,1000,0.3);        //Drives forwards

        //Shoots the 3 Balls
        drive(-1000,-1000,-1000,-1000,0.3);        //Drives backwards
        drive(-900,-900,900,900,0.4);//Turns left
        shoot(0.6, 3, 1, 1, 0.95, 0); //Shoots intaked ballz


        //Gets out of the square
        drive(-100,-100,-100,-100,0.3);        //Drives backwards

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


        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);


        while (opModeIsActive() && frontleft.isBusy() && backleft.isBusy() && frontright.isBusy() && backright.isBusy()) {
            idle();
        }
    }

    //Shooting method
    private void shoot(double shooterPower, long time, double intakePower, double ballBooterPOS, double hoodPOS, double turretringPWR) {
        intake.setPower(intakePower);
        bottomshooter.setPower((-1 * shooterPower));
        topshooter.setVelocity(1 * 2800 * shooterPower);
        ballBooter.setPosition(ballBooterPOS);
        hood.setPosition(hoodPOS);
        turretring.setPower(turretringPWR);

        sleep(time * 1000);

        bottomshooter.setPower(0);
        topshooter.setPower(0);
        intake.setPower(0);
        turretring.setPower(0);
    }


}
