package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "FullAuto")
public class FullAuto extends LinearOpMode {
    // Live-tunable in FTC Dashboard (launcher/intake powers & gate positions live under Robot)
    public static double DRIVE_POWER = 0.65;
    public static double STRAFE_POWER = 0.5;
    public static double DRIVE_FORWARD_INCHES = 10.0;
    public static double STRAFE_INCHES = 45.0;
    public static double DRIVE_TIMEOUT_S = 5.0;
    public static double SPINUP_TIMEOUT_S = 5.0;
    public static double STRAFE_TIMEOUT_S = 5.0;
    public static long GATE_OPEN_MS = 300;
    public static long GATE_CLOSED_MS = 600;

    private final Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        robot.runDriveInstructions(DRIVE_POWER, DRIVE_FORWARD_INCHES, DRIVE_FORWARD_INCHES, DRIVE_TIMEOUT_S);

        // Get the launcher up to speed using shared Robot.POLLEN_LAUNCHER_POWER
        robot.startPollenLauncher(Robot.POLLEN_LAUNCHER_POWER);
        // Wait for the launcher to get up to speed and then open gate
        robot.waitForPollenGateOpen(SPINUP_TIMEOUT_S);
        // allow one ball to go through
        sleep(GATE_OPEN_MS);
        // Second ball
        gateClosedWaitToOpen();

        robot.setTransferIntake(Robot.INTAKE_POWER);
        // Third ball
        gateClosedWaitToOpen();
        // Fourth ball
        gateClosedWaitToOpen();

        robot.setTransferIntake(0);
        robot.stopPollenLauncher();

        robot.runStrafeInstructions(STRAFE_POWER, STRAFE_INCHES, STRAFE_TIMEOUT_S);
    }

    private void gateClosedWaitToOpen() {
        robot.pollenGateClose();
        sleep(GATE_CLOSED_MS);
        robot.pollenGateOpen();
        sleep(GATE_OPEN_MS);
    }
}