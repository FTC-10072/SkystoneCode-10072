package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous(name = "Red Side Tray", group = "Auto")
public class redSideTray extends LinearOpMode {
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

        driveTrain.strafeToDistance(-22,.8,4);
        driveTrain.driveToDistance(-30,5,1);
        arm.dropgripper();
        sleep(500);
        driveTrain.driveToDistance(35,5,.5);
        arm.raisegripper();
        driveTrain.strafeToDistance(22,.8,5);
        driveTrain.driveToDistance(-28,3,.5);
        driveTrain.turnToDegree(115, 1, 3);
        driveTrain.driveToDistance(-26,3,.6);
        driveTrain.driveToDistance(3,1,.6);


        sleep(500);
        driveTrain.turnToDegree(-35,1,4);
        sleep(500);
        driveTrain.driveToDistance(26,5,1);

        driveTrain.stop();
    }
}
