//IMPORTING LIBRARIES
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class RedFarAutonomous extends LinearOpMode {
    //DEFINING MOTORS
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor shooter; //Shooting Motor
    public Servo trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor transferSystem; //The motors in the chamber
    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;
    @Override
    public void runOpMode() {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        trigger = hardwareMap.get(Servo.class, "shooter left");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        transferSystem = hardwareMap.get(DcMotor.class, "transfer");

        //RESTARTING ENCODERS
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //trigger.setMode(Servo.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferSystem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        //SETTING NEW VARIABLES
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;

        //THE AUTO ITSELF
        waitForStart();

        drive(1000,1000,1000,1000,0.25); //Drives
        drive(1000,1000,-1000,-1000,0.1); //Turns some direction
        drive(-1000,1000,1000,-1000,0.1); //Strafes some direction
    }
    //DRIVE FUNCTION
    private void drive(int leftBackTarget,int leftFrontTarget, int rightBackTarget,int rightFrontTarget, double speed) {
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

        while(opModeIsActive() && frontleft.isBusy() && backleft.isBusy() && frontright.isBusy() && backright.isBusy()){
            idle();
        }
    }
}
