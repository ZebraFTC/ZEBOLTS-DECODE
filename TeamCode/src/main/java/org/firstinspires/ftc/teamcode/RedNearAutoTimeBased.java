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

    private static ElapsedTime timer = new ElapsedTime();

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

        //SETTING MOTOR DIRECTIONS
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        //THE AUTO ITSELF
        waitForStart();
        driveWhileSpinUp(0.5, -0.5, -0.5, -0.5, -0.5, -0.47, 0, 0); //drives back at beginning
        shootClose();
        driveWhileSpinUp(0.23 , 0.5, -0.5, 0.5, -0.5, 0.15, 0, 0); //turns after shooting three balls
        driveWhileSpinUp(0.75, 0.5, -0.5, -0.5, 0.5, 0.15, 0, 0);//straifs to other 3 balls
        driveWhileSpinUp(2.2, 0.25, 0.25, 0.25, 0.25, 0.15, -1, 0.25); //intake next three balls
        driveWhileSpinUp(0.4, -0.5, -0.5, -0.5, -0.5, 0,-1, 0); //go back after getting three balls
        driveWhileSpinUp(0.15, -0.5, 0.5, -0.5, 0.5, 0, 0, 0); //turns after picking  up three balls
        driveWhileSpinUp(0.76, -0.5, 0.5, 0.5, -0.5, -0.47, 0, 0); //straif after turn with three balls
        shootClose();
        driveWhileSpinUp(0.23, 0.5, -0.5, 0.5, -0.5, 0, 0, 0);
        driveWhileSpinUp(1.32, 0.5, -0.5, -0.5, 0.5, 0, 0, 0);
        driveWhileSpinUp(2.45, 0.5, 0.5, 0.5, 0.5, 0, -1, 0.25);
        driveWhileSpinUp(0.75, -0.5, -0.5, -0.5, -0.5, -0.47, -1, 0);
        driveWhileSpinUp(1.62, -0.5, 0.5, 0.5, -0.5, -0.47, 0, 0);
        driveWhileSpinUp(0.1, -0.5, 0.5, -0.5, 0.5, -0.47, 0, 0);
        shootClose();
    }

    //DRIVE FUNCTION
    private void driveWhileSpinUp(double time, double flPower, double frPower, double blPower, double brPower, double shooterPower, double intakePower, double triggerPower) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            frontleft.setPower(flPower);
            backleft.setPower(blPower);
            frontright.setPower(frPower);
            backright.setPower(brPower);
            intake.setPower(intakePower);
            trigger.setPower(triggerPower);
            bottomshooter.setPower(shooterPower);
            topshooter.setPower(shooterPower);

        }
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        intake.setPower(0);
    }

    private void shootClose() {
        timer.reset();
        long startTime = System.currentTimeMillis();
        long waitDuration = 3000; // Wait for 2 seconds
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
        while (opModeIsActive() && timer.seconds() < 6.5) {
            intake.setPower(-0.75);
            trigger.setPower(0.75);
        }
        intake.setPower(0);
        trigger.setPower(0);
        bottomshooter.setPower(0);
        topshooter.setPower(0);
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

