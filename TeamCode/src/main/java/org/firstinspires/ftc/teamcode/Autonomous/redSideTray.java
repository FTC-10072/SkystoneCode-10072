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

        driveTrain.strafeToDistance(-32,.8,4);
        driveTrain.driveToDistance(-26,4,1);
        arm.dropgripper();
        sleep(300);
        driveTrain.driveToDistance(37,5,.5);
        arm.raisegripper();

        driveTrain.stop();
    }
}
