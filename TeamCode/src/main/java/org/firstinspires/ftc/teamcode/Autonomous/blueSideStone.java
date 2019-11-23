package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.colorSensor;

@Autonomous(name = "Blue side Stone", group = "Auto")
public class blueSideStone extends LinearOpMode {
    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private Arm arm = new Arm();
    colorSensor colorSensor = new colorSensor();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        colorSensor.init(robot, this);
        arm.init(robot, this);
        waitForStart();

        /*colorSensor.lightOn();


        driveTrain.driveToStone();
        colorSensor.lightOff();*/
        driveTrain.strafeToDistance(10,1,5);
    }
}
