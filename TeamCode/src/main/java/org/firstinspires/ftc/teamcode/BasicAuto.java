package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous", group = "Robot")
public class BasicAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        robot.runDriveInstructions(0.5, 20, 20, 20);

        sleep(500);

        robot.startPollenLauncher(0.5);
        robot.startNectarLauncher(0.5);
        robot.setTransferIntake(0.5);

        robot.runLaunchersForSeconds(5.0);

        robot.stopPollenLauncher();
        robot.stopNectarLauncher();
        robot.setTransferIntake(0);
    }
}