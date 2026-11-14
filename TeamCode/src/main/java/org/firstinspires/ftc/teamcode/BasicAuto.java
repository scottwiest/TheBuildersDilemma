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
        }
    }
