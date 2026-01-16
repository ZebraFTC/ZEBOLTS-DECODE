//IMPORTING LIBRARIES
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueNearAutonomousTimeBAsed extends LinearOpMode {
    //DEFINING MOTORS
    public DcMotor frontleft; //Wheel
    public DcMotor frontright; //Wheel
    public DcMotor backleft; //Wheel
    public DcMotor backright; //Wheel
    public Servo transfer; //The thing that launches the ball into the shooting system
    public DcMotor intake; //The intake
    public DcMotor bottomshooter; //The motors in the chamber
    public Servo hood;
    public DcMotor turretring;
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
        transfer = hardwareMap.get(Servo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomshooter = hardwareMap.get(DcMotor.class, "shooter 1");
        hood = hardwareMap.get(Servo.class, "angle changer");
        turretring = hardwareMap.get(DcMotor.class, "turret ring");

        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        //THE AUTO ITSELF
        waitForStart();
        driveWhileSpinUp(0.75, -0.5, -0.5, -0.5, -0.5, 0.62, -0.8,1,0.9,0); //drives back at beginning
        driveWhileSpinUp(4, 0, 0, 0, 0, 0.61, -0.8,1,0.9,0);
        driveWhileSpinUp(2, 0, 0, 0, 0, 0.61, -0.8,0.85,0.82,0);
        driveWhileSpinUp(0.22, -0.5, 0.5, -0.5, 0.5, 0, 0,1,0.9,0); //turns after shooting three balls
        driveWhileSpinUp(0.74, -0.5, 0.5, 0.5, -0.5, 0, 0,1,0.9,0);//straifs to other 3 balls
        driveWhileSpinUp(2.5, 0.25, 0.25, 0.25, 0.25, 0, -0.8,1,0.9,0); //intake next three balls
        driveWhileSpinUp(0.45, -0.5, -0.5, -0.5, -0.5, 0, -0.8,1,0.9,0); //go back after getting three balls
        driveWhileSpinUp(0.22, 0.5, -0.5, 0.5, -0.5, 0.65, -0.8,1,0.9,0); //turns after picking  up three balls
        driveWhileSpinUp(0.76, 0.5, -0.5, -0.5, 0.5, 0.65, -0.8,1,0.9,0); //straif after turn with three balls
        driveWhileSpinUp(4, 0, 0, 0, 0, 0.6, -0.8,1,0.9,0);
        driveWhileSpinUp(2, 0, 0, 0, 0, 0.6, -0.8,0.85,0.82,0);
        driveWhileSpinUp(0.75, -0.5, 0.5, 0.5, -0.5, 0, 0,1,0.9,0);
    }

    //DRIVE FUNCTION
    private void driveWhileSpinUp(double time, double flPower, double frPower, double blPower, double brPower, double shooterPower, double intakePower, double transfer1, double hood1, double turretring1) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            frontleft.setPower(flPower);
            backleft.setPower(blPower);
            frontright.setPower(frPower);
            backright.setPower(brPower);
            intake.setPower(intakePower);
            bottomshooter.setPower(-1 * shooterPower);
            transfer.setPosition(transfer1);
            hood.setPosition(hood1);
            turretring.setPower(turretring1);


        }
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        intake.setPower(0);
    }

    private void shootClose(long waitDurationMilli) {
        timer.reset();
        long startTime = System.currentTimeMillis();
        long waitDuration = waitDurationMilli; // Wait for 3 seconds
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
    }
}

