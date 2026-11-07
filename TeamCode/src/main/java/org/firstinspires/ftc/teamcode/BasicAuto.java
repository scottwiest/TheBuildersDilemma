package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous", group="Robot")

public class BasicAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        while (opModeIsActive()) {
            robot.runDriveInstructions(0.5, 20, 20, 20);

            sleep(500);

            robot.setPollenLauncher(0.5);
            robot.setNectarLauncher(0.5);
            robot.setTransferIntake(0.5);
            robot.nectarGateOpen();
            robot.pollenGateOpen();

            sleep(5000);

            robot.setPollenLauncher(0);
            robot.setNectarLauncher(0);
            robot.setTransferIntake(0);
            robot.nectarGateClose();
            robot.pollenGateClose();
        }
    }
}