package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private LinearOpMode currentOpMode;

    private Servo gripperServo;
    private DcMotor intakeMotor1, intakeMotor2, worstMotor;
    //private Servo catchServo;



    public void init(HardwareRobot robot, LinearOpMode opMode){

        currentOpMode = opMode;
        gripperServo = robot.gripperServo;
        intakeMotor1 = robot.intakeMotor1;
        intakeMotor2 = robot.intakeMotor2;
        worstMotor   = robot.worstMotor;
        gripperServo.setPosition(1);




    }

    public void runIntake(double power){
        intakePower(power);

    }


    public void moveArmUp(double speed){
        speed = Math.abs(speed);
        worstMotor.setPower(speed);
    }

    public void moveArmDown(double speed){
        speed = Math.abs(speed);
        worstMotor.setPower(-speed);
    }


    public void reverseIntake(double power){
        intakePower(-power);
    }

    public void stopIntake(){
        intakePower(0);
    }

    public void intakePower(double power){
        intakeMotor1.setPower(power);
        intakeMotor2.setPower(power);
    }


    public void dropgripper(){
        gripperServo.setPosition(.5);
    }
    public void raisegripper(){
        gripperServo.setPosition(1);
    }







}
