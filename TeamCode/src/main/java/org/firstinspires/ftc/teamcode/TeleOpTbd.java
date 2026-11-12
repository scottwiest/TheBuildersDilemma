package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class TeleOpTbd extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        // Initialize the robot hardware using the OpMode's hardwareMap
        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            boolean relativeDriveFlag = true;

            // Drive control
            if (relativeDriveFlag) {
                robot.setRelativeDrive(forward, right, rotate);
            } else {
                robot.setDrivePower(forward, right, rotate);
            }

            // Pollen Servo control
            if (gamepad1.leftBumperWasPressed()) {
                robot.pollenGateOpen();
                sleep(500);
                robot.pollenGateClose();
            }
            // Pollen Launcher control
            if (gamepad1.leftTriggerWasPressed()) {
                robot.setPollenLauncher(0.2);
            }
            if (gamepad1.leftTriggerWasReleased()) {
                robot.setPollenLauncher(0);
            }

            // Nectar Servo Control
            if (gamepad1.rightBumperWasPressed()) {
                robot.nectarGateOpen();
                sleep(500);
                robot.nectarGateClose();
            }
            // Nectar Launcher Control
            if (gamepad1.rightTriggerWasPressed()) {
                robot.setNectarLauncher(0.2);
            }
            if (gamepad1.rightTriggerWasReleased()) {
                robot.setNectarLauncher(0);
            }


            if (gamepad1.dpadUpWasPressed()) {
                robot.setTransferIntake(1);
            }
            if (gamepad1.dpadUpWasReleased()) {
                robot.setTransferIntake(0);
            }
            if (gamepad1.dpadDownWasPressed()) {
                robot.setTransferIntake(-1);
            }
            if (gamepad1.dpadDownWasReleased()) {
                robot.setTransferIntake(0);
            }

            robot.logLauncherVelocity();
        }
    }
}
