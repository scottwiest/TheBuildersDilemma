package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp1DriverPov", group = "B")
public class TeleOp1DriverPov extends LinearOpMode {

    private final Robot robot = new Robot(this);

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
                robot.startPollenLauncher(Robot.POLLEN_LAUNCHER_POWER);
            }
            if (gamepad1.leftBumperWasPressed()) {
                robot.stopPollenLauncher();
            }

            // Nectar Launcher & Gate Control:
            // Right trigger starts spin-up (opens gate at >=98%, closes at <=96%);
            // Right bumper closes gate and stops launcher.
            if (gamepad1.rightTriggerWasPressed()) {
                robot.startNectarLauncher(Robot.NECTAR_LAUNCHER_POWER);
            }
            if (gamepad1.rightBumperWasPressed()) {
                robot.stopNectarLauncher();
            }

            robot.updateLaunchers();


            if (gamepad1.yWasPressed()) {
                robot.setTransferIntake(Robot.INTAKE_POWER);
            }
            if (gamepad1.yWasReleased()) {
                robot.setTransferIntake(0);
            }
            if (gamepad1.aWasPressed()) {
                robot.setTransferIntake(Robot.INTAKE_REVERSE_POWER);
            }
            if (gamepad1.aWasReleased()) {
                robot.setTransferIntake(0);
            }

            robot.logLauncherVelocity();
        }
    }
}