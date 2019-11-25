package org.firstinspires.ftc.teamcode.RobotClasses;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HDriveHardwareRobot {
    public DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, horiMotor, armMotor;
    public Servo trayServo1, trayServo2, armServo1, armServo2;
    public BNO055IMU imu;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HDriveHardwareRobot(){}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // setup motors
        leftFrontMotor = hwMap.get(DcMotor.class, "left_motor_1");
        leftBackMotor = hwMap.get(DcMotor.class, "left_motor_2");
        rightFrontMotor = hwMap.get(DcMotor.class, "right_motor_1");
        rightBackMotor = hwMap.get(DcMotor.class, "right_motor_2");
        horiMotor      = hwMap.get(DcMotor.class, "hori_motor");
        armMotor       = hwMap.get(DcMotor.class, "arm_motor");

        initMotor(leftFrontMotor, -1);
        initMotor(leftBackMotor,-1);
        initMotor(rightFrontMotor,1);
        initMotor(rightBackMotor,1);
        initMotor(horiMotor, 1);
        initMotor(armMotor, 1);

        // setup Servos

        trayServo1 = hwMap.get(Servo.class, "tray_servo_1");
        trayServo2 = hwMap.get(Servo.class, "tray_servo_2");

        armServo1  = hwMap.get(Servo.class, "arm_motor_1");
        armServo2  = hwMap.get(Servo.class, "arm_motor_2");

        trayServo1.setPosition(.5);
        trayServo2.setPosition(.5);

        armServo1.setPosition(0);
        armServo2.setPosition(1);




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
}
