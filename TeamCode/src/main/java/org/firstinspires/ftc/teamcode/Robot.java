package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Robot {
    // Declare hardware members
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor transferIntake = null;
    private Servo nectarGate = null;
    private Servo pollenGate = null;
    private LauncherController pollenController = null;
    private LauncherController nectarController = null;
    private final LinearOpMode opMode;
    private IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    static final double WHEEL_TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double WHEEL_COUNTS_PER_INCH = (WHEEL_TICKS_PER_REV) / (WHEEL_DIAMETER_INCHES * 3.14159);

    // GoBilda 5203 (6000 RPM, 1:1 ratio) Launcher Motor Constants
    public static final double LAUNCHER_TICKS_PER_REV = 28.0;
    public static final double LAUNCHER_MAX_RPM = 6000.0;
    // Max ticks/sec = (6000 RPM / 60 sec) * 28 ticks/rev = 2800 ticks/sec
    public static final double LAUNCHER_MAX_TICKS_PER_SEC = (LAUNCHER_MAX_RPM / 60.0) * LAUNCHER_TICKS_PER_REV;

    // REV Hub Velocity PIDF starting coefficients for GoBilda 5203 6000 RPM:
    // F = 32767 / max_ticks_per_sec = 32767 / 2800 ≈ 11.7025
    // P = 0.1 * F ≈ 1.17 (can be tuned higher, e.g., 5.0 - 12.0, for faster recovery between shots)
    // I = 0.1 * P ≈ 0.117 (set to 0.0 if flywheel overshoots after spin-up)
    // D = 0.0
    public static final double LAUNCHER_F = Short.MAX_VALUE / LAUNCHER_MAX_TICKS_PER_SEC;
    public static final double LAUNCHER_P = 0.1 * LAUNCHER_F;
    public static final double LAUNCHER_I = 0.1 * LAUNCHER_P;
    public static final double LAUNCHER_D = 0.0;

    public static final PIDFCoefficients POLLEN_PIDF_COEFFS =
            new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);
    public static final PIDFCoefficients NECTAR_PIDF_COEFFS =
            new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);

    // Gate velocity hysteresis thresholds (open at >= 95%, close if speed drops to <= 90%)
    public static final double GATE_OPEN_THRESHOLD = 0.95;
    public static final double GATE_CLOSE_THRESHOLD = 0.90;

    // Constructor
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hardwareMap) {

        // Map devices to the names configured on the Driver Station / Control Hub
        frontLeftWheel = hardwareMap.get(DcMotor.class, "front_left_wheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "back_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "back_right_wheel");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectar_launcher");
        transferIntake = hardwareMap.get(DcMotor.class, "transfer_intake");
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");

        // Set motor directions (adjust according to your drivetrain orientation)
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        pollenLauncher.setDirection(DcMotorEx.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        transferIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero-power behavior
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        configureLaunchersPIDF();
        pollenController = new LauncherController(pollenLauncher, this::pollenGateOpen, this::pollenGateClose);
        nectarController = new LauncherController(nectarLauncher, this::nectarGateOpen, this::nectarGateClose);

        // Reset and stop motors
        frontLeftWheel.setPower(0);
        backLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backRightWheel.setPower(0);
        pollenLauncher.setVelocity(0);
        nectarLauncher.setVelocity(0);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void configureLaunchersPIDF() {
        pollenLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);
        nectarLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);

        pollenLauncher.setMode(RunMode.RUN_USING_ENCODER);
        nectarLauncher.setMode(RunMode.RUN_USING_ENCODER);

        pollenLauncher.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, POLLEN_PIDF_COEFFS);
        nectarLauncher.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, NECTAR_PIDF_COEFFS);
    }

    public void setRunUsingEncoder() {
        frontLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(RunMode.RUN_USING_ENCODER);

        configureLaunchersPIDF();
    }

    // Helper methods keep OpModes clean
    public void setDrivePower(double forward, double right, double rotate) {

        double max;

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        double scalingFactor = 0.65;
        // Send calculated power to wheels
        frontLeftWheel.setPower(frontLeftPower * scalingFactor);
        frontRightWheel.setPower(frontRightPower * scalingFactor);
        backLeftWheel.setPower(backLeftPower * scalingFactor);
        backRightWheel.setPower(backRightPower * scalingFactor);
    }

    public void runDriveInstructions(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutS
    ) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget =
                    frontLeftWheel.getCurrentPosition() + (int) (leftInches * WHEEL_COUNTS_PER_INCH);
            newRightFrontTarget =
                    frontRightWheel.getCurrentPosition() + (int) (rightInches * WHEEL_COUNTS_PER_INCH);
            newLeftBackTarget = backLeftWheel.getCurrentPosition() + (int) (leftInches * WHEEL_COUNTS_PER_INCH);
            newRightBackTarget =
                    backRightWheel.getCurrentPosition() + (int) (rightInches * WHEEL_COUNTS_PER_INCH);
            frontLeftWheel.setTargetPosition(newLeftFrontTarget);
            frontRightWheel.setTargetPosition(newRightFrontTarget);
            backLeftWheel.setTargetPosition(newLeftBackTarget);
            backRightWheel.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            frontLeftWheel.setMode(RunMode.RUN_TO_POSITION);
            frontRightWheel.setMode(RunMode.RUN_TO_POSITION);
            backLeftWheel.setMode(RunMode.RUN_TO_POSITION);
            backRightWheel.setMode(RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftWheel.setPower(Math.abs(speed));
            frontRightWheel.setPower(Math.abs(speed));
            backLeftWheel.setPower(Math.abs(speed));
            backRightWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy()
                            && backRightWheel.isBusy())) {
                opMode.sleep(10);
            }

            // Stop all motion;
            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backLeftWheel.setPower(0);
            backRightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
            frontRightWheel.setMode(RunMode.RUN_USING_ENCODER);
            backLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
            backRightWheel.setMode(RunMode.RUN_USING_ENCODER);

            opMode.sleep(100);   // pause after each move.
        }
    }

    public void leftStrafe() {
        frontRightWheel.setPower(1);
        frontLeftWheel.setPower(-1);
        backRightWheel.setPower(-1);
        backLeftWheel.setPower(1);
    }

    public void rightStrafe() {
        frontRightWheel.setPower(-1);
        frontLeftWheel.setPower(1);
        backRightWheel.setPower(1);
        backLeftWheel.setPower(-1);
    }


    /**
     * Sets the pollen launcher speed as a fraction of max velocity (-1.0 to 1.0)
     * using built-in DcMotorEx PIDF velocity control.
     */
    public void setPollenLauncher(double power) {
        pollenLauncher.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
    }

    /**
     * Sets the pollen launcher target velocity directly in encoder ticks per second.
     */
    public void setPollenLauncherVelocity(double ticksPerSec) {
        pollenLauncher.setVelocity(ticksPerSec);
    }

    /**
     * Sets the pollen launcher target speed in RPM (up to 6000 RPM for GoBilda 5203).
     */
    public void setPollenLauncherRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * LAUNCHER_TICKS_PER_REV;
        pollenLauncher.setVelocity(ticksPerSec);
    }

    /**
     * Sets the nectar launcher speed as a fraction of max velocity (-1.0 to 1.0)
     * using built-in DcMotorEx PIDF velocity control.
     */
    public void setNectarLauncher(double power) {
        nectarLauncher.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
    }

    /**
     * Sets the nectar launcher target velocity directly in encoder ticks per second.
     */
    public void setNectarLauncherVelocity(double ticksPerSec) {
        nectarLauncher.setVelocity(ticksPerSec);
    }

    /**
     * Sets the nectar launcher target speed in RPM (up to 6000 RPM for GoBilda 5203).
     */
    public void setNectarLauncherRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * LAUNCHER_TICKS_PER_REV;
        nectarLauncher.setVelocity(ticksPerSec);
    }

    public double getPollenLauncherVelocity() {
        return pollenLauncher.getVelocity();
    }

    public double getPollenLauncherRPM() {
        return (pollenLauncher.getVelocity() / LAUNCHER_TICKS_PER_REV) * 60.0;
    }

    public double getNectarLauncherVelocity() {
        return nectarLauncher.getVelocity();
    }

    public double getNectarLauncherRPM() {
        return (nectarLauncher.getVelocity() / LAUNCHER_TICKS_PER_REV) * 60.0;
    }

    public void logLauncherVelocity() {
        opMode.telemetry.addData("Pollen Launcher", "%.1f ticks/s (%.0f RPM) | Gate: %s",
                getPollenLauncherVelocity(), getPollenLauncherRPM(),
                pollenController.isGateOpen() ? "OPEN" : "CLOSED");
        opMode.telemetry.addData("Nectar Launcher", "%.1f ticks/s (%.0f RPM) | Gate: %s",
                getNectarLauncherVelocity(), getNectarLauncherRPM(),
                nectarController.isGateOpen() ? "OPEN" : "CLOSED");
        opMode.telemetry.update();
    }

    public void setTransferIntake(double power) {
        transferIntake.setPower(power);
    }

    public void pollenGateOpen() {
        pollenGate.setPosition(0);
    }

    public void pollenGateClose() {
        pollenGate.setPosition(0.11);
    }

    public void nectarGateOpen() {
        nectarGate.setPosition(0.12);
    }

    public void nectarGateClose() {
        nectarGate.setPosition(0.0);
    }

    /**
     * Manages non-blocking velocity spin-up and 95%-open / 90%-close gate hysteresis for a launcher.
     */
    private static class LauncherController {
        private final DcMotorEx motor;
        private final Runnable openGate;
        private final Runnable closeGate;
        private double targetPower = 0.0;
        private boolean gateOpen = false;

        LauncherController(DcMotorEx motor, Runnable openGate, Runnable closeGate) {
            this.motor = motor;
            this.openGate = openGate;
            this.closeGate = closeGate;
        }

        void start(double power) {
            if (power == 0.0) {
                stop();
                return;
            }
            targetPower = power;
            if (!gateOpen) {
                closeGate.run();
            }
            motor.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
        }

        void stop() {
            targetPower = 0.0;
            gateOpen = false;
            closeGate.run();
            motor.setVelocity(0);
        }

        void update() {
            if (targetPower == 0.0) {
                return;
            }
            double targetVel = Math.abs(targetPower * LAUNCHER_MAX_TICKS_PER_SEC);
            double currentVel = Math.abs(motor.getVelocity());

            if (!gateOpen && currentVel >= targetVel * GATE_OPEN_THRESHOLD) {
                openGate.run();
                gateOpen = true;
            } else if (gateOpen && currentVel <= targetVel * GATE_CLOSE_THRESHOLD) {
                closeGate.run();
                gateOpen = false;
            }
        }

        boolean isGateOpen() {
            return gateOpen;
        }
    }

    public void startPollenLauncher(double power) {
        pollenController.start(power);
    }

    public void stopPollenLauncher() {
        pollenController.stop();
    }

    public void startNectarLauncher(double power) {
        nectarController.start(power);
    }

    public void stopNectarLauncher() {
        nectarController.stop();
    }

    /**
     * Call every loop in TeleOp (or Auto) to automatically open launcher gates at >= 95% target
     * velocity and close them if velocity drops to <= 90%.
     */
    public void updateLaunchers() {
        pollenController.update();
        nectarController.update();
    }

    /**
     * Helper for Autonomous: maintains 95%/90% gated launching for the given duration in seconds.
     */
    public void runLaunchersForSeconds(double durationSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < durationSeconds) {
            updateLaunchers();
            opMode.sleep(10);
        }
    }

    public void setRelativeDrive(
            double forward,
            double right,
            double rotate
    ) {
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (opMode.gamepad1.aWasPressed()) {
            imu.resetYaw();
        }
        // If you press the B button, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (opMode.gamepad1.b) {
            setDrivePower(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        setDrivePower(newForward, newRight, rotate);
    }
}
