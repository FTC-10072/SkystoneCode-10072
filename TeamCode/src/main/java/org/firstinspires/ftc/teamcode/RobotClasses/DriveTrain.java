package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotClasses.colorSensor;

public class DriveTrain {

    private LinearOpMode currentOpMode;
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, horiMotor;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    Orientation angles;
    Acceleration gravity;
    private double globalAngle;

    colorSensor rSense = new colorSensor();

    // Ku = 0.105
    // Tu = .866

    private double TURN_P = 0.041;
    private double TURN_I = 0.0;
    private double TURN_D = 0.00315;
    private static final double MAX_DRIVE_SPEED = 0.9;
    private static final double MAX_TURN_SPEED = 0.7;
    private static final double GAIN = 0.1;
    private static final double DEADBAND = 0.15;

    private static final AxesOrder axes = AxesOrder.ZYX;

    private static final int COUNTS_PER_REV = 1120; // count / rev
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.141592; // distance / rev
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE;

    public DriveTrain(){}

    public void init(HardwareRobot robot, LinearOpMode opMode){
        currentOpMode   = opMode;
        leftFrontMotor  = robot.leftFrontMotor;
        leftBackMotor   = robot.leftBackMotor;
        rightFrontMotor = robot.rightFrontMotor;
        rightBackMotor  = robot.rightBackMotor;
        horiMotor       = robot.horiMotor;


        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = robot.imu;



        resetAngle();
    }
    

    public void turnWithEncoder(double input){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        leftFrontMotor.setPower(input);
        leftBackMotor.setPower(input);
        rightFrontMotor.setPower(-input);
        rightBackMotor.setPower(-input);
    }

    // drive to specified distance with specified precision
    @SuppressLint("Assert")
    public boolean driveToDistance(double targetDistance, double timeout,double power){
        if(currentOpMode.opModeIsActive()) {
            assert timeout > 0;
            // set new target position
            int newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * targetDistance);
            int newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * targetDistance);
            int newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * targetDistance);
            int newRightBackTarget = rightBackMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * targetDistance);

            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset time
            ElapsedTime time = new ElapsedTime();
            // loop until diff is within precision or timeout
            while (currentOpMode.opModeIsActive() && time.seconds() < timeout
                    && leftFrontMotor.isBusy() && leftBackMotor.isBusy()
                    && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
                // set power with correction
                setLeftPower(power, MAX_DRIVE_SPEED, checkDirection());
                setRightPower(power, MAX_DRIVE_SPEED);
            }
            // check if finished
            if (time.seconds() > timeout) return false;
            // stop motors
            setLeftPower(0.0, 0.0);
            setRightPower(0.0, 0.0);
            // return that it ended under timeout
            return true;
        }
        return false;
    }

    public boolean strafeToDistance(int distance, double speed, double timeout){
        if (currentOpMode.opModeIsActive()) {
            int cpr = (int) (COUNTS_PER_INCH * .9);
            int move = distance * cpr;
            double mechDrift = .9;

            int newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + move;
            int newLeftBackTarget = leftBackMotor.getCurrentPosition() - move;
            int newRightFrontTarget = rightFrontMotor.getCurrentPosition() - move;
            int newRightBackTarget = rightBackMotor.getCurrentPosition() + move;

            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ElapsedTime time = new ElapsedTime();

            while (currentOpMode.opModeIsActive() && time.seconds() < timeout
                    && leftFrontMotor.isBusy() && leftBackMotor.isBusy()
                    && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
                // set power with correction
                leftFrontMotor.setPower(mechDrift * speed);
                rightFrontMotor.setPower(speed);
                leftBackMotor.setPower(speed);
                rightBackMotor.setPower(mechDrift * speed);
            }
            if (time.seconds() > timeout){
                return false;
            }
            return true;

        }
        return false;






    }
    //probably a better turning alg
    // turn to specific degree. for use in Auto
    // positive is to the left, negative is to the right
    @SuppressLint("Assert")
    public boolean turnToDegree(double targetAngle, double precision, double timeout){
        // assert values
        assert precision > 0;
        assert timeout > 0;
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // make sure targetAngle is bounded
        if(targetAngle > 180.0) targetAngle = 179.9;
        if(targetAngle < -180.0) targetAngle = -179.9;
        // reset angle
        resetAngle();
        // loop values
        double currentAngle = -getAngle();
        double diff = currentAngle - targetAngle;
        double power;
        double totalDiffs = 0.0;
        double previousDiff = diff;
        double derivative;
        double period = 0.05;
        // reset time
        ElapsedTime time = new ElapsedTime();

        while(currentOpMode.opModeIsActive() && time.seconds() < timeout
                && Math.abs(diff) > precision){
            // integrate
            totalDiffs += diff * period;
            // derivative
            derivative = (diff - previousDiff)/period;
            // set power
            power = diff * TURN_P + totalDiffs * TURN_I + derivative * TURN_D;
            setLeftPower(power, MAX_TURN_SPEED);
            setRightPower(-power, MAX_TURN_SPEED);
            // update values
            currentAngle = -    getAngle();
            previousDiff = diff;
            diff = currentAngle - targetAngle;
            currentOpMode.telemetry.addData("target", targetAngle);
            currentOpMode.telemetry.addData("current", currentAngle);
            currentOpMode.telemetry.addData("power", power);
            currentOpMode.telemetry.addData("diff", diff);
            currentOpMode.telemetry.addData("total", totalDiffs);
            currentOpMode.telemetry.addData("deriv", derivative);
            currentOpMode.telemetry.update();
            // sleep
            currentOpMode.sleep((int)(1000*period));
        }
        // check if finished
        if(time.seconds() > timeout){
            stop();
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
        // stop motors
        stop();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
    }


    public void driveToStone(){

        //while red is greater than 15 keep on moving
        while (rSense.checkColor()){
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
        }

    }




    public void arcadeDrive(double move, double turn, double strafe){
        move = boundValue(move);
        move = deadband(move, DEADBAND);
        turn = boundValue(turn);
        turn = deadband(turn, DEADBAND);

        // Combine drive and turn for blended motion.
        double left  = move + turn;
        double right = move - turn;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        horiMotor.setPower(strafe);

        // Output the safe vales to the motor drives.
        setLeftPower(left, MAX_DRIVE_SPEED);
        setRightPower(right, MAX_DRIVE_SPEED);

    }



    public void mecanumDrive(double x1, double y1, double x2){

        double r = Math.hypot(x1,y1);

        double Angle = Math.atan2(x1,y1) + Math.PI/4;
        double rot = x2;

        final double lf =  r * Math.cos(Angle) + rot;
        final double lb =  r * Math.sin(Angle) + rot;
        final double rf =  r * Math.sin(Angle) - rot;
        final double rb =  r * Math.cos(Angle) - rot;

        leftFrontMotor.setPower(lf);
        leftBackMotor.setPower(lb);
        rightFrontMotor.setPower(rf);
        rightBackMotor.setPower(rb);


    }


    public void mecanumDriveFieldCentric(double x1, double y1, double x2){
        double lx =  x1 , ly = y1;
        double r  =  Math.hypot(x1, y1);

        double currAngle = Math.atan2(lx, ly);

        double current = Math.toRadians(getAngle());


        double Angle = currAngle + current;
        double rot = x2;

        final double lf = r * Math.cos(Angle) + rot;
        final double lb = r * Math.sin(Angle) + rot;
        final double rf = r * Math.sin(Angle) - rot;
        final double rb = r * Math.cos(Angle) - rot;

        currentOpMode.telemetry.addData("lf", lf);
        currentOpMode.telemetry.addData("lb", lb);
        currentOpMode.telemetry.addData("rf", rf);
        currentOpMode.telemetry.addData("rb", rb);
        currentOpMode.telemetry.update();


        leftFrontMotor.setPower(lf);
        leftBackMotor.setPower(lb);
        rightFrontMotor.setPower(rf);
        rightBackMotor.setPower(rb);


    }

    public void stop(){
        leftFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
    }

    // normalize power and set left motors to that power (with correction)
    private void setLeftPower(double power, double maxPower, double correction){
        power = boundValue(power) * maxPower;
        leftFrontMotor.setPower(power + correction);
        leftBackMotor.setPower(power + correction);
    }

    // normalize power and set left motors
    private void setLeftPower(double power, double maxPower){
        power = boundValue(power) * maxPower;
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
    }

    // normalize power and set right motors to that power
    private void setRightPower(double power, double maxPower){
        power = boundValue(power) * maxPower;
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    // bound value to -1.0, 1.0
    private double boundValue(double value){
        if(value > 1.0) return 1.0;
        else if(value < -1.0) return -1.0;
        else return value;
    }

    // return 0 if abs(value) is less than band
    private double deadband(double value, double band){
        if(-band < value && value < band) return 0;
        return value;
    }

    // set all motor modes
    private void setMode(DcMotor.RunMode mode){
        leftFrontMotor.setMode(mode);
        leftBackMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        rightBackMotor.setMode(mode);
    }

    // reset the gyro angle
    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, axes, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // get the current gyro angle
    // positive is to the left, negative is to the right
    public double getAngle(){
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, axes, AngleUnit.DEGREES);
        double deltaAngle = newAngles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180){
            deltaAngle += 360;
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = newAngles;

        return globalAngle;
    }

    // get correction factor for driving forward
    private double checkDirection(){
        double angle = getAngle();
        return angle * GAIN;
    }

    public void changePID(double p, double i, double d){
        TURN_P = Math.abs(p);
        TURN_I = Math.abs(i);
        TURN_D = Math.abs(d);
        currentOpMode.telemetry.addData("PID", "%f, %f, %f", TURN_P, TURN_I, TURN_D);
        currentOpMode.telemetry.update();
    }
}
