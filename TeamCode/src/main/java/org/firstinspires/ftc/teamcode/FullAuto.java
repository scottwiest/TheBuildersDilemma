package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "FullAuto")
public class FullAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        robot.runDriveInstructions(0.5, 10, 10, 5);

        // 1. Spin up pollen launcher and wait until it reaches >= 98% speed and opens the gate
        robot.startPollenLauncher(0.4);
        robot.waitForPollenGateOpen(5.0);
        sleep(300);
        robot.pollenGateClose();
        sleep(600);
        robot.pollenGateOpen();

        // 2. Only turn on transfer intake AFTER the pollen gate is open, then launch 4 balls
        robot.setTransferIntake(1);
        sleep(300);
        robot.pollenGateClose();
        sleep(600);
        robot.pollenGateOpen();
        robot.waitForPollenShots(2, 5.0);

        // 3. Stop intake and launcher
        robot.setTransferIntake(0);
        robot.stopPollenLauncher();

        robot.runStrafeInstructions(0.5, 45, 5);
    }
}