package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous(name = "Blue side Tray", group = "Auto")
public class JustMoveForwardPlease extends LinearOpMode {
    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private Arm arm = new Arm();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        arm.init(robot, this);
        waitForStart();

        //driveTrain.driveToDistance(24*Math.sqrt(2.0),7);
        driveTrain.strafeToDistance(18,.8,3);
        driveTrain.driveToDistance(-35,3,1);
        arm.dropgripper();
        sleep(500);
        //driveTrain.turnToDegree(-10,.1,2);
        driveTrain.turnToDegree(-45,.5,2);
        driveTrain.driveToDistance(20,2,.5);

        driveTrain.turnToDegree(-45,.5,2);
        arm.raisegripper();
        driveTrain.driveToDistance(-5,2,.8);
        driveTrain.driveToDistance(55,3,.8);

        /*driveTrain.strafeToDistance(-40, .8, 3);
        driveTrain.driveToDistance(-17,2,.8);
        driveTrain.strafeToDistance(17,.8,3);
        //driveTrain.turnToAngle(30,.5);
        driveTrain.turnToAngle(90, -.5);
        driveTrain.turnToAngle(90,-.5);
        //driveTrain.driveToDistance(27,5,.8);
        driveTrain.turnToDegree(-90, .25,3);
        driveTrain.stop();*/
    }
}
