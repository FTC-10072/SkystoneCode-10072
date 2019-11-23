package org.firstinspires.ftc.teamcode.RobotClasses;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class colorSensor {

    private LinearOpMode currentOpMode;

    private ModernRoboticsI2cColorSensor rSensor;



    public void init(HardwareRobot robot, LinearOpMode opMode){

        currentOpMode = opMode;
        rSensor = robot.rSensor;



    }


    public void lightOn(){
        rSensor.enableLed(true);
    }

    public void lightOff(){
        rSensor.enableLed(false);
    }

    public boolean checkColor(){
        if (rSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) > 6 && rSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) < 10){
            return true;
        }
        return false;
    }

    /*public boolean checkColor(){
        if (rSensor.red() > 50){
            currentOpMode.telemetry.addData("redValue", rSensor.red());
            return true;

        }

        return false;

    }*/
}
