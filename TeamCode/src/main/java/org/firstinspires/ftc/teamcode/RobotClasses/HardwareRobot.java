package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;




public class HardwareRobot {

    public DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor,intakeMotor1, intakeMotor2, worstMotor, horiMotor;

    public Servo gripperServo, move1Servo, move2Servo;
    public BNO055IMU imu;
    public ModernRoboticsI2cColorSensor rSensor;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot(){}

    public void initMotor(DcMotor motor, int direction){
        if (direction > 0){
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        else if (direction < 0){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        // setup motors
        leftFrontMotor     = hwMap.get(DcMotor.class,"left_motor_1");
        leftBackMotor      = hwMap.get(DcMotor.class,"left_motor_2");
        rightFrontMotor    = hwMap.get(DcMotor.class,"right_motor_1");
        rightBackMotor     = hwMap.get(DcMotor.class,"right_motor_2");
        intakeMotor1       = hwMap.get(DcMotor.class, "intake_motor_1");
        intakeMotor2       = hwMap.get(DcMotor.class, "intake_motor_2");
        worstMotor         = hwMap.get(DcMotor.class, "worst_motor");
        horiMotor          = hwMap.get(DcMotor.class, "hori_motor");




        initMotor(leftFrontMotor, -1);
        initMotor(leftBackMotor,-1);
        initMotor(rightFrontMotor,1);
        initMotor(rightBackMotor,1);
        initMotor(intakeMotor1, 1);
        initMotor(intakeMotor2,-1);
        initMotor(worstMotor,-1);
        initMotor(horiMotor, 1);

        /*
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        worstMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horiMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // positive is clockwise/up, negative is anticlockwise/down


        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        worstMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horiMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        worstMotor.setPower(0);
        horiMotor.setPower(0);


        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        worstMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        worstMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horiMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        // set up servos

        gripperServo = hwMap.get(Servo.class, "gripper_servo");
        move1Servo = hwMap.get(Servo.class, "move_1_servo");
        move2Servo = hwMap.get(Servo.class, "move_2_servo");


        gripperServo.setPosition(1);
        move1Servo.setPosition(.5);
        move2Servo.setPosition(.5);

        rSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "rSense");



        // set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
