package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous", group = "Robot")
public class LaunchingAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        robot.runDriveInstructions(0.5, 10, 10, 20);

        robot.setPollenLauncher(0.4);

        sleep(5000);

        robot.setTransferIntake(1);

        robot.pollenGateOpen();
        sleep(500);
        robot.pollenGateClose();

        sleep(1000);

        robot.pollenGateOpen();
        sleep(500);
        robot.pollenGateClose();

        sleep(1000);

        robot.pollenGateOpen();
        sleep(500);
        robot.pollenGateClose();

        sleep(1000);

        robot.pollenGateOpen();
        sleep(500);
        robot.pollenGateClose();

        sleep(1000);

        robot.setPollenLauncher(0);
        robot.setTransferIntake(0);
    }
}