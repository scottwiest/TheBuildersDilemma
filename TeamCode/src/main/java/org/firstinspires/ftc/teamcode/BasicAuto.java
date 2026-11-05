package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous", group="Robot")

public class BasicAuto extends LinearOpMode{

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.setRunUsingEncoder();

        waitForStart();

        while(opModeIsActive()){

            //Run to launching positions
            robot.runDriveInstructions(0,0,0,0);

            //Launch pollen
            launchPollen();

            //Run to parking position
            robot.runDriveInstructions(0,0,0,0);
        }
    }

    public void launchPollen(power){
        robot.setPollenLauncherPower(power);
        robot.pollenGateOpen();
        sleep(500);
        robot.pollenGateClose();
        robot.setPollenLauncherPower(0);
    }
     public void launchNectar(power){
        robot.setNectarLauncher(power);
        robot.nectarGateOpen();
        sleep(500);
        robot.nectarGateClose();
        robot.setNectarLauncher(0);
    }
}
