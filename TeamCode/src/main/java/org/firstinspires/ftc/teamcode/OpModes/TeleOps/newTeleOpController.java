package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.newArm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HDriveHardwareRobot;

@TeleOp(name = "TeleOp Controller", group = "TeleOp")

public class newTeleOpController extends LinearOpMode{

    private HDriveHardwareRobot robot = new HDriveHardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();

    private newArm arm = new newArm();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);

        arm.init(robot, this);

        double leftStickY;
        double rightStickX;
        double leftStickX;
        int direction = 1;

        leftStickY  = direction * gamepad1.left_stick_y;
        leftStickX  = direction * gamepad1.left_stick_x;
        rightStickX = direction * gamepad1.right_stick_x;

        if (gamepad1.x){
            direction *= -1;
        }

        driveTrain.arcadeDrive(leftStickY,rightStickX,leftStickX);

        if(gamepad2.dpad_down){
            arm.dropGripper();
        }
        if(gamepad2.dpad_up){
            arm.raiseGripper();
        }

        if(gamepad2.right_stick_y> .2){
            arm.MoveArm(.5);
        }
        else if(gamepad2.right_stick_y < -.2){
            arm.MoveArm(-.5);
        }
        else{
            arm.MoveArm(0);
        }

        if (gamepad2.a){
            arm.extendGripper();
        }

        if (gamepad2.b){
            arm.retractGripper();
        }

        if(gamepad2.x){
            arm.openGripper();
        }
        if (gamepad2.y){
            arm.closeGripper();
        }
    }


}
