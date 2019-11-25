package org.firstinspires.ftc.teamcode.RobotClasses;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class newArm {
    private LinearOpMode currentOpMode;
    private DcMotor armMotor;
    private Servo trayServo1, trayServo2, armServo1, armServo2;

    public void init(HDriveHardwareRobot robot, LinearOpMode opMode){
        currentOpMode = opMode;
        trayServo1 = robot.trayServo1;
        trayServo2 = robot.trayServo2;
        armMotor   = robot.armMotor;
        armServo1  = robot.armServo1;
        armServo2  = robot.armServo2;

        trayServo1.setPosition(.5);
        trayServo2.setPosition(.5);
    }

    public void dropGripper(){
        trayServo1.setPosition(1);
        trayServo2.setPosition(0);
    }

    public void raiseGripper(){
        trayServo1.setPosition(.5);
        trayServo2.setPosition(.5);
    }

    public void MoveArm(double power){
        armMotor.setPower(power);
    }

    public void closeGripper(){
        armServo2.setPosition(.2);
    }
    public void openGripper(){
        armServo2.setPosition(1);
    }
    public void extendGripper(){
        armServo1.setPosition(1);
    }
    public void retractGripper(){
        armServo1.setPosition(0);
    }



}
