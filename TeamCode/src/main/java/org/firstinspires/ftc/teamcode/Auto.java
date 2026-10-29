package org.firstinspires.ftc.teamcode;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode{

    private final Robot robot = new Robot(this);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized in Cloud Workspace");
        telemetry.update();

        waitForStart();

        //Leave perimeter wall and move forward for 1.5 seconds
        setDrivePower(1,0,0);
        sleep(1500);

        //Move left into parking zone
        setDrivePower(0,-1,0);
        sleep(2500);
    }

}