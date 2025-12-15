//IMPORTING LIBRARIES
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedNearAutoTimeBased extends LinearOpMode {
    //DEFINING MOTORS
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public DcMotor shooter; //Shooting Motor
    public Servo trigger; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor bottomshooter; //The motors in the chamber
    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;
    private int shooterPos;
    private int intakePos;
    private int topShooterPos;
    private int triggerPos;

    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        //DEFINING HARDWARE MAP ***
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        trigger = hardwareMap.get(Servo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");

        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        //THE AUTO ITSELF
        waitForStart();
        driveWhileSpinUp(0.5, -0.5, -0.5, -0.5, -0.5, 0, 0); //drives back at beginning
        shootClose();
        driveWhileSpinUp(0.21, 0.5, -0.5, 0.5, -0.5, 0.15, 0); //turns after shooting three balls
        driveWhileSpinUp(0.7, 0.5, -0.5, -0.5, 0.5, 0.15, 0);//straifs to other 3 balls
        driveWhileSpinUp(2.5, 0.25, 0.25, 0.25, 0.25, 0.15, -1); //intake next three balls
        driveWhileSpinUp(0.4, -0.5, -0.5, -0.5, -0.5, -0.5, 0); //go back after getting three balls
        driveWhileSpinUp(0.15, -0.5, 0.5, -0.5, 0.5, 0, 0); //turns after picking  up three balls
        driveWhileSpinUp(0.76, -0.5, 0.5, 0.5, -0.5, 0, 0); //straif after turn with three balls
        driveWhileSpinUp(0.5, 0, 0, 0, 0, -0.47, 0);
        shootClose();
        driveWhileSpinUp(0.7, 0.5, -0.5, -0.5, 0.5, 0, 0);
    }

    //DRIVE FUNCTION
    private void driveWhileSpinUp(double time, double flPower, double frPower, double blPower, double brPower, double shooterPower, double intakePower) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            frontleft.setPower(flPower);
            backleft.setPower(blPower);
            frontright.setPower(frPower);
            backright.setPower(brPower);
            intake.setPower(intakePower);
            trigger.setPosition(0.1);
            bottomshooter.setPower(shooterPower);


        }
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        intake.setPower(0);
    }

    private void shootClose() {
        bottomshooter.setPower(-0.55);
        timer.reset();
        long startTime = System.currentTimeMillis();
        long waitDuration = 4000; // Wait for 2 seconds
        long elapsedTime = 0;
        while (elapsedTime < waitDuration) {
            elapsedTime = System.currentTimeMillis() - startTime;
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        while (opModeIsActive() && timer.seconds() < 0.2) {
            intake.setPower(-0.75);
        }
        while (opModeIsActive() && timer.seconds() < 0.3) {
            trigger.setPosition(0.97);
        }
        while (opModeIsActive() && timer.seconds() < 0.5) {
            trigger.setPosition(0.1);
        }
        while (opModeIsActive() && timer.seconds() < 0.25) {
            trigger.setPosition(0.97);
        }
        while (opModeIsActive() && timer.seconds() < 0.25) {
            trigger.setPosition(0.1);
        }
        while (opModeIsActive() && timer.seconds() < 1) {
            intake.setPower(-0.75);
        }
        while (opModeIsActive() && timer.seconds() < 0.25) {
            trigger.setPosition(0.97);
        }
        while (opModeIsActive() && timer.seconds() < 0.25) {
            trigger.setPosition(0.1);
        }
        while (opModeIsActive() && timer.seconds() < 1) {
            intake.setPower(-0.75);
        }
        intake.setPower(0);
        trigger.setPosition(0);
        bottomshooter.setPower(0);
    }
    private void intake() {
        while (opModeIsActive() && timer.seconds() < 0.3) {
            frontleft.setPower(-0.5);
            backleft.setPower(-0.5);
            frontright.setPower(0.5);
            backright.setPower(0.5);
        }
    }
}

