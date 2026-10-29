package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "TeleOp")
public class TeleOpTbd extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        // Initialize the robot hardware using the OpMode's hardwareMap
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            boolean relativeDriveFlag = false;

            // Drive control
            if (relativeDriveFlag) {
                telemetry.addLine("Press A to reset the rotation");
                telemetry.addLine("Hold left bumper to drive in robot relative");
                telemetry.addLine("The left joystick sets the robot direction");
                telemetry.addLine("Moving the right joystick left and right turns the robot");

                robot.setRelativeDrive(forward, right, rotate, false, false);
            } else {
                robot.setDrivePower(forward, right, rotate);
            }

            // Servo control
            if (gamepad1.left_trigger_pressed) {
                robot.nectarGateOpen();
                sleep(500);
                robot.nectarGateClose();
            }
            if (gamepad1.right_trigger_pressed) {
                robot.pollenGateOpen();
                sleep(500);
                robot.pollenGateClose();
            }

            // Launcher control
            if (gamepad1.left_bumper) {
                robot.setPollenLauncherPower(0.5);
            }
            if (gamepad1.leftBumperWasReleased()) {
                robot.setPollenLauncherPower(0);
            }
            if (gamepad1.right_bumper) {
                robot.setNectarLauncher(0.5);

                if (gamepad1.rightBumperWasReleased()) {
                    robot.setNectarLauncher(0);
                }
            }
        }
    }
}
