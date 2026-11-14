package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "DummyAuto")
public class DummyAuto extends LinearOpMode {
    public static double DRIVE_POWER = 0.65;
    public static double STRAFE_POWER = 0.5;
    public static double DRIVE_FORWARD_INCHES = 10.0;
    public static double STRAFE_INCHES = 45.0;
    public static double DRIVE_TIMEOUT_S = 5.0;
    public static double STRAFE_TIMEOUT_S = 5.0;

    private final Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        robot.runDriveInstructions(DRIVE_POWER, DRIVE_FORWARD_INCHES, DRIVE_FORWARD_INCHES, DRIVE_TIMEOUT_S);

        robot.runStrafeInstructions(STRAFE_POWER, STRAFE_INCHES, STRAFE_TIMEOUT_S);
    }
}