package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@Config
@SuppressWarnings("unused")
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
    static final double WHEEL_COUNTS_PER_INCH = (WHEEL_TICKS_PER_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final long LOOP_POLL_MS = 10;

    // GoBilda 5203 (6000 RPM, 1:1 ratio) Launcher Motor Constants
    public static final double LAUNCHER_TICKS_PER_REV = 28.0;
    public static final double LAUNCHER_MAX_RPM = 6000.0;
    // Max ticks/sec = (6000 RPM / 60 sec) * 28 ticks/rev = 2800 ticks/sec
    public static final double LAUNCHER_MAX_TICKS_PER_SEC = (LAUNCHER_MAX_RPM / 60.0) * LAUNCHER_TICKS_PER_REV;

    // --- Live-tunable FTC Dashboard (@Config) variables (must be public static & non-final) ---
    public static double DRIVE_SCALING = 0.85;
    public static double STRAFE_MULTIPLIER = 1.2;

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_REVERSE_POWER = -1.0;

    public static double POLLEN_LAUNCHER_POWER = 0.405;
    public static double NECTAR_LAUNCHER_POWER = 0.42;

    public static double POLLEN_GATE_OPEN_POS = 0.0;
    public static double POLLEN_GATE_CLOSE_POS = 0.11;
    public static double NECTAR_GATE_OPEN_POS = 0.12;
    public static double NECTAR_GATE_CLOSE_POS = 0.0;

    // REV Hub Velocity PIDF coefficients for GoBilda 5203 6000 RPM flywheel:
    // F = 32767 / max_ticks_per_sec ≈ 11.70 (increase slightly, e.g. 12.5, if steady RPM sits below target)
    // P = 60.0 (strong recovery boost when a ball drops wheel speed; tune between 40.0 and 100.0)
    // I = 0.0  (kept at 0 so spin-up doesn't overshoot and launch the 1st ball higher than the rest)
    // D = 0.0  (kept at 0 to avoid amplifying 28-tick encoder noise)
    public static double LAUNCHER_F = Short.MAX_VALUE / LAUNCHER_MAX_TICKS_PER_SEC;
    public static double LAUNCHER_P = 6.0;
    public static double LAUNCHER_I = 0.1;
    public static double LAUNCHER_D = 0.0;

    public static double GATE_OPEN_THRESHOLD = 0.99;
    public static double GATE_CLOSE_THRESHOLD = 0.975;
    public static double GATE_MIN_CLOSE_SECONDS = 0.20;
    public static long POST_MOVE_PAUSE_MS = 100;

    // Track last applied PIDF values so live FTC Dashboard edits push to the REV Hub automatically
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    // Constructor
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hardwareMap) {
        // Send telemetry to both Driver Hub and FTC Dashboard (for live graphs & values)
        opMode.telemetry = new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

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
        frontLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        transferIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero-power behavior
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(RunMode.RUN_USING_ENCODER);

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

        applyCurrentPIDF();
    }

    private void applyCurrentPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);
        pollenLauncher.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, coeffs);
        nectarLauncher.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, coeffs);
        appliedP = LAUNCHER_P;
        appliedI = LAUNCHER_I;
        appliedD = LAUNCHER_D;
        appliedF = LAUNCHER_F;
    }

    private void syncPIDFIfChanged() {
        if (LAUNCHER_P != appliedP
                || LAUNCHER_I != appliedI
                || LAUNCHER_D != appliedD
                || LAUNCHER_F != appliedF) {
            applyCurrentPIDF();
        }
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

        // Send calculated power to wheels (DRIVE_SCALING is live-editable in FTC Dashboard)
        frontLeftWheel.setPower(frontLeftPower * DRIVE_SCALING);
        frontRightWheel.setPower(frontRightPower * DRIVE_SCALING);
        backLeftWheel.setPower(backLeftPower * DRIVE_SCALING);
        backRightWheel.setPower(backRightPower * DRIVE_SCALING);
    }

    /**
     * Drives forward/backward or turns in place using encoder targets for left and right sides.
     */
    public void runDriveInstructions(
            double power,
            double leftInches,
            double rightInches,
            double timeoutS
    ) {
        runDriveInstructions(power, leftInches, rightInches, leftInches, rightInches, timeoutS);
    }

    /**
     * Strafes sideways using encoders: positive rightInches strafes RIGHT, negative strafes LEFT.
     */
    public void runStrafeInstructions(
            double power,
            double rightInches,
            double timeoutS
    ) {
        double d = rightInches * STRAFE_MULTIPLIER;
        runDriveInstructions(power, d, -d, -d, d, timeoutS);
    }

    /**
     * Moves all four wheels to individual encoder distance targets (in inches).
     */
    private void runDriveInstructions(
            double power,
            double frontLeftInches,
            double frontRightInches,
            double backLeftInches,
            double backRightInches,
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
                    frontLeftWheel.getCurrentPosition() + (int) (frontLeftInches * WHEEL_COUNTS_PER_INCH);
            newRightFrontTarget =
                    frontRightWheel.getCurrentPosition() + (int) (frontRightInches * WHEEL_COUNTS_PER_INCH);
            newLeftBackTarget =
                    backLeftWheel.getCurrentPosition() + (int) (backLeftInches * WHEEL_COUNTS_PER_INCH);
            newRightBackTarget =
                    backRightWheel.getCurrentPosition() + (int) (backRightInches * WHEEL_COUNTS_PER_INCH);
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
            frontLeftWheel.setPower(Math.abs(power));
            frontRightWheel.setPower(Math.abs(power));
            backLeftWheel.setPower(Math.abs(power));
            backRightWheel.setPower(Math.abs(power));

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
                opMode.sleep(LOOP_POLL_MS);
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

            opMode.sleep(POST_MOVE_PAUSE_MS);   // pause after each move.
        }
    }

    /**
     * Sets the pollen launcher speed as a fraction of max velocity (-1.0 to 1.0)
     * using built-in DcMotorEx PIDF velocity control.
     */
    public void setPollenLauncher(double power) {
        pollenLauncher.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
    }

    /**
     * Sets the nectar launcher speed as a fraction of max velocity (-1.0 to 1.0)
     * using built-in DcMotorEx PIDF velocity control.
     */
    public void setNectarLauncher(double power) {
        nectarLauncher.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
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

    private void logSingleLauncher(String name, DcMotorEx motor, LauncherController controller) {
        double currentRpm = (motor.getVelocity() / LAUNCHER_TICKS_PER_REV) * 60.0;
        double targetRpm = controller.getTargetPower() * LAUNCHER_MAX_RPM;
        opMode.telemetry.addData(name + " RPM", Math.round(currentRpm));
        opMode.telemetry.addData(name + " Target RPM", Math.round(targetRpm));
        opMode.telemetry.addData(name + " Delta", Math.round(currentRpm - targetRpm));
        opMode.telemetry.addData(name + " Gate", "%s",
                controller.isGateOpen() ? "OPEN" : "CLOSED");
    }

    public void logLauncherVelocity() {
        logSingleLauncher("Pollen", pollenLauncher, pollenController);
        logSingleLauncher("Nectar", nectarLauncher, nectarController);
        opMode.telemetry.update();
    }

    public void setTransferIntake(double power) {
        transferIntake.setPower(power);
    }

    public void pollenGateOpen() {
        pollenGate.setPosition(POLLEN_GATE_OPEN_POS);
    }

    public void pollenGateClose() {
        pollenGate.setPosition(POLLEN_GATE_CLOSE_POS);
    }

    public void nectarGateOpen() {
        nectarGate.setPosition(NECTAR_GATE_OPEN_POS);
    }

    public void nectarGateClose() {
        nectarGate.setPosition(NECTAR_GATE_CLOSE_POS);
    }

    public boolean isPollenGateOpen() {
        return pollenController.isGateOpen();
    }

    /**
     * Manages non-blocking velocity spin-up, 98%-open / 96%-close gate hysteresis,
     * and a 200ms gate-close hold time so back-to-back balls feed one at a time.
     */
    private static class LauncherController {
        private final DcMotorEx motor;
        private final Runnable openGate;
        private final Runnable closeGate;
        private final ElapsedTime closeHoldTimer = new ElapsedTime();
        private double targetPower = 0.0;
        private boolean gateOpen = false;
        private boolean holdingGateClosed = false;

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
            if (targetPower == power) {
                return;
            }
            targetPower = power;
            holdingGateClosed = false;
            if (!gateOpen) {
                closeGate.run();
            }
            motor.setVelocity(power * LAUNCHER_MAX_TICKS_PER_SEC);
        }

        void stop() {
            targetPower = 0.0;
            gateOpen = false;
            holdingGateClosed = false;
            closeGate.run();
            motor.setVelocity(0);
        }

        void update() {
            if (targetPower == 0.0) {
                return;
            }
            if (holdingGateClosed && closeHoldTimer.seconds() >= GATE_MIN_CLOSE_SECONDS) {
                holdingGateClosed = false;
            }

            double targetVel = Math.abs(targetPower * LAUNCHER_MAX_TICKS_PER_SEC);
            double currentVel = Math.abs(motor.getVelocity());

            if (!gateOpen && !holdingGateClosed && currentVel >= targetVel * GATE_OPEN_THRESHOLD) {
                gateOpen = true;
            } else if (gateOpen && currentVel <= targetVel * GATE_CLOSE_THRESHOLD) {
                gateOpen = false;
                holdingGateClosed = true;
                closeHoldTimer.reset();
            }

            // Apply current open/close servo position (picks up live FTC Dashboard edits immediately)
            if (gateOpen) {
                openGate.run();
            } else {
                closeGate.run();
            }
        }

        boolean isGateOpen() {
            return gateOpen;
        }

        double getTargetPower() {
            return targetPower;
        }

        void syncRunningPower(double configuredPower) {
            if (targetPower != 0.0 && targetPower != configuredPower) {
                targetPower = configuredPower;
                motor.setVelocity(configuredPower * LAUNCHER_MAX_TICKS_PER_SEC);
            }
        }
    }

    public void startPollenLauncher() {
        pollenController.start(POLLEN_LAUNCHER_POWER);
    }

    public void startPollenLauncher(double power) {
        pollenController.start(power);
    }

    public void stopPollenLauncher() {
        pollenController.stop();
    }

    public void startNectarLauncher() {
        nectarController.start(NECTAR_LAUNCHER_POWER);
    }

    public void startNectarLauncher(double power) {
        nectarController.start(power);
    }

    public void stopNectarLauncher() {
        nectarController.stop();
    }

    /**
     * Call every loop in TeleOp (or Auto) to automatically sync live Dashboard power/PIDF changes,
     * open launcher gates at >= GATE_OPEN_THRESHOLD, and close them at <= GATE_CLOSE_THRESHOLD.
     */
    public void updateLaunchers() {
        syncPIDFIfChanged();
        pollenController.syncRunningPower(POLLEN_LAUNCHER_POWER);
        nectarController.syncRunningPower(NECTAR_LAUNCHER_POWER);
        pollenController.update();
        nectarController.update();
    }

    /**
     * Helper for Autonomous: waits until the given launcher reaches >= 98% speed and opens its gate
     * (or until timeoutSeconds elapses).
     */
    private void waitForGateOpen(LauncherController controller, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive()
                && timer.seconds() < timeoutSeconds
                && !controller.isGateOpen()) {
            controller.update();
            logLauncherVelocity();
            opMode.sleep(LOOP_POLL_MS);
        }
    }

    public void waitForPollenGateOpen(double timeoutSeconds) {
        waitForGateOpen(pollenController, timeoutSeconds);
    }

    public void waitForNectarGateOpen(double timeoutSeconds) {
        waitForGateOpen(nectarController, timeoutSeconds);
    }

    /**
     * Helper for Autonomous: maintains velocity-gated launching for the given duration in seconds.
     */
    public void runLaunchersForSeconds(double durationSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < durationSeconds) {
            updateLaunchers();
            opMode.sleep(LOOP_POLL_MS);
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

        // Third, convert back to Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        setDrivePower(newForward, newRight, rotate);
    }
}
