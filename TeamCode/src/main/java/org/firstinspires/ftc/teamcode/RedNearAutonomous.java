//IMPORTING LIBRARIES
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedNearAutonomous extends LinearOpMode {
    //DEFINING MOTORS
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor shooter; //Shooting Motor
    public DcMotor trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor bottomshooter; //The motors in the chamber
    public DcMotor topshooter; //The motors in the chamber
    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;
    private int shooterPos;
    private int intakePos;
    private int topShooterPos;
    private int triggerPos;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        trigger = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        topshooter = hardwareMap.get(DcMotor.class, "shooter 2");

        //RESTARTING ENCODERS
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //trigger.setMode(Servo.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topshooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        //SETTING NEW VARIABLES
        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;
        shooterPos = 0;
        intakePos = 0;
        topShooterPos = 0;

        //THE AUTO ITSELF
        waitForStart();

        drive(-500, -500, -500, -500, 0.5);
        telemetry.addLine("Driving Backward"); //"Printing" to the driver hub
        // bottomshooter.setPower(0.25);
        // topshooter.setPower(-0.25);
        shoot(100, 0.25);
        /*
        --TEMPLATES--`w-[
        drive(1000,1000,1000,1000,0.25);        //Drives
        drive(1000,1000,-1000,-1000,0.1);       //Turns some direction
        drive(-1000,1000,1000,-1000,0.1);       //Strafes some direction
        shoot(1000, 0.66, false);              //Intakes/Transfers Balls
        shoot(1000, 0.66, true);               //Shoots 1 Ball
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

    private void shoot(int shootTarget, double speed) {

        intakePos -= shootTarget;
        triggerPos += shootTarget;
        shooterPos -= shootTarget;

        intake.setTargetPosition(intakePos);
        trigger.setTargetPosition(triggerPos);
        bottomshooter.setTargetPosition(shooterPos);
        topshooter.setTargetPosition(shooterPos);

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trigger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomshooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topshooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPower(0.5);
        trigger.setPower(1);
        bottomshooter.setPower(speed);
        topshooter.setPower(-speed);

        /*long startTime = System.currentTimeMillis();
        long waitDuration = 3000; // Wait for 3 seconds
        long elapsedTime = 0;
        while (elapsedTime < waitDuration) {
            elapsedTime = System.currentTimeMillis() - startTime;
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }*/

        while (opModeIsActive() && bottomshooter.isBusy() && intake.isBusy() && topshooter.isBusy() && trigger.isBusy()) {
            idle();
        }
    }
}
