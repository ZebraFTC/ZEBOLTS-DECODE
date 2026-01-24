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
        waitForStart(); //Hrithik we can run the shoot method to intake and not shoot if desired

        //SECTION 1: SHOOTING PRELOADED BALLS
        turnTurret(0.5,-270);
        drive(-700,-700,-700,-700,0.5);
        drive(-900,900,900,-900,0.5);
       //shoot
        drive(-1600,1600,1600,-1600,0.5);

        //SECTION 2: GETTING MORE BALLS
        //intake
        drive(800,800,800,800,0.5);

        //SECTION 3: OPENING GATE
        drive(-300,-300,-300,-300,0.5);
        drive(300,-300,-300,300,0.5);
        drive(450,450,450,450,0.5);
        sleep(2000);

        //SECTION 4: SHOOT AND INTAKE MORE
        drive(-1300,-1300,-1300,-1300,0.5);
        drive(700,-700,-700,700,0.5);
        //shoot
        //intake
        drive(1200,1200,1200,1200,0.5);
        drive(-1000,-1000,-1000,-1000,0.5);
        drive(600,-600,-600,600,0.5);
        //shoot

        //SECTION 5: INTAKE AND SHOOT THE LAST SET
        drive(-2300,2300,2300,-2300,0.5);
        //intake
        drive(1000,1000,1000,1000,0.5);
        drive(-1000,-1000,-1000,-1000,0.5);
        drive(1500,-1500,-1500,1500,0.5);
        //shoot













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

        sleep(500);
    }

    //Shooting method
    private void shoot(double shooterPower, long time, double intakePower, double ballBooterPOS, double hoodPOS) {
        intake.setPower(intakePower);
        bottomshooter.setPower((-1 * shooterPower));
        topshooter.setVelocity(1 * 2800 * shooterPower);
        ballBooter.setPosition(ballBooterPOS);
        hood.setPosition(hoodPOS);
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
}
