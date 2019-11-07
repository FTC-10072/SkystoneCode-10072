package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "TeleOp Controller", group = "TeleOp")
public class TeleOpController extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();

    private Arm arm = new Arm();

    /*
    TASK          | CONTROLLER | WHERE
    drive train   | 1          | joysticks
    intake motion | 1          | dpad
    elevator      | 2          | dpad
    claw          | 2          | y
    ratchet       | 2          | b
    intake spin   | 2          | bumpers
    intake stop   | 2          | triggers
    move arm up   | 2          | x
    move arm down | 2          | a

     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);

        arm.init(robot, this);

        double leftStickY;
        double rightStickX;
        double leftStickX;
        int direction = 1;





        waitForStart();

        while(opModeIsActive()){
            leftStickY  = direction * gamepad1.left_stick_y;
            leftStickX  = direction * gamepad1.left_stick_x;
            rightStickX = direction * gamepad1.right_stick_x;

            if (gamepad1.a && gamepad1.b){
                direction *= -1;
            }

            // drive train
            driveTrain.mecanumDrive(leftStickX, leftStickY, rightStickX);

            //move the gripper

            if (gamepad1.a){
                arm.raisegripper();
            }

            if (gamepad1.b){
                arm.dropgripper();
            }


        }
    }
}
