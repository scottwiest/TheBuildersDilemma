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

            robot.setDrivePower(forward, right, rotate);

            // Pollen Launcher & Gate Control:
            // Left trigger starts spin-up (opens gate at >=98%, closes at <=96%);
            // Left bumper closes gate and stops launcher.
            if (gamepad1.leftTriggerWasPressed()) {
                robot.startPollenLauncher(0.405);
            }
            if (gamepad1.leftBumperWasPressed()) {
                robot.stopPollenLauncher();
            }

            // Nectar Launcher & Gate Control:
            // Right trigger starts spin-up (opens gate at >=98%, closes at <=96%);
            // Right bumper closes gate and stops launcher.
            if (gamepad1.rightTriggerWasPressed()) {
                robot.startNectarLauncher(0.42);
            }
            if (gamepad1.rightBumperWasPressed()) {
                robot.stopNectarLauncher();
            }

            robot.updateLaunchers();


            if (gamepad1.aWasPressed()) {
                robot.setTransferIntake(1);
            }
            if (gamepad1.aWasReleased()) {
                robot.setTransferIntake(0);
            }
            if (gamepad1.bWasPressed()) {
                robot.setTransferIntake(-1);
            }
            if (gamepad1.bWasReleased()) {
                robot.setTransferIntake(0);
            }

            robot.logLauncherVelocity();
        }
    }
}
