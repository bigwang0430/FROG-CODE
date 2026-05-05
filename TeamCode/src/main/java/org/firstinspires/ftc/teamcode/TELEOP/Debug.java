package org.firstinspires.ftc.teamcode.TELEOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "Debug")
public class Debug extends OpMode {

    private Motor l1, l2, intake, transfer;
    private ServoEx hood, tiltl, tiltr, t1, t2, gate, lights;
    private DigitalChannel bb;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private Follower follower;
    private GamepadEx g1, g2;
    private int lastPosition;
    private double lastTime, previousRPM, RPM;
    @Override
    public void init() {
        t1 = new ServoEx(hardwareMap, "t1", 360);
        t2 = new ServoEx(hardwareMap, "t2", 360);
        t2.setInverted(true);
        t1.setInverted(true);
        t1.set(180);
        t2.set(180);

        l1 = new Motor(hardwareMap, "l1", 28, 6000);
        l2 = new Motor(hardwareMap, "l2", 28, 6000);
        l1.setRunMode(Motor.RunMode.RawPower);
        l2.setRunMode(Motor.RunMode.RawPower);
        l2.setInverted(true);
        l1.setInverted(false);
        l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        gate = new ServoEx(hardwareMap, "gate");
        gate.set(globals.gate.close);

        intake = new Motor(hardwareMap, "intake");
        intake.stopAndResetEncoder();
        intake.resetEncoder();

        lights = new ServoEx(hardwareMap, "lights");
        lights.set(0);


        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(false);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300);
        hood.set(0);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        launchPIDF.setTolerance(100);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(60, 84, Math.PI/2));
    }

    @Override
    public void loop() {
        gate();
        intake();
        launch();
        RPM();
        telemetry.update();
        follower.update();
    }

    private void gate() {
        if (g2.getButton(GamepadKeys.Button.DPAD_UP)) {
            gate.set(globals.gate.open);
        } else {
            gate.set(globals.gate.close);
        }
    }

    private void intake() {
        if (g2.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(1);
            transfer.set(1);
        } else {
            intake.set(0);
            transfer.set(0);
        }
    }

    private void launch() {
        if (g2.getButton(GamepadKeys.Button.CROSS)) {
            launchPIDF.setSetPoint(globals.tuning.targetRPM);
            double launchPower = launchPIDF.calculate(RPM);
            if (RPM < 400) {
                l1.set(0.6);
                l2.set(0.6);
            } else {
                l1.set(launchPower + globals.launcher.kv * globals.tuning.targetRPM + globals.launcher.ks);
                l2.set(launchPower + globals.launcher.kv * globals.tuning.targetRPM + globals.launcher.ks);
            }
        } else {
            l1.set(0);
            l2.set(0);
        }
    }

    public void RPM() {
        double currentTime = getRuntime();
        int currentPosition = l2.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = -(revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }



}
