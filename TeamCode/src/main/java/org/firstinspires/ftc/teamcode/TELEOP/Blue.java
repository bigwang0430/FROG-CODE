package org.firstinspires.ftc.teamcode.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.GLOBALS.globals;

import java.util.List;
import java.util.Objects;

@TeleOp (name = "Blue")
public class Blue extends OpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(20, 20);
    private double offset;

    private Motor l1, l2, intake, transfer;
    private ServoEx hood, gate, lights;
    private ServoEx t1, t2;

    private GamepadEx g1, g2;
    private Follower follower;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private ElapsedTime timer = new ElapsedTime();
    private DigitalChannel bb;
    private enum aimMode {
        odo,
        cam
    } private aimMode currentAimMode = aimMode.odo;
    private enum intakeState {
        idle,
        intaking,
        launching,
        eject

    } private intakeState currentIntakeState = intakeState.idle;
    private String robotLocation = "No Zone";

    private boolean tagReady;
    private double lastTime, launchPower, RPM, previousRPM, dist, turretAng, targetRPM, hoodAngle, leftY, leftX, tagAng;
    private int lastPosition;
    private boolean prevCross1, prevTriggerR, prevTriggerL;
    private boolean autoAim = true;
    private boolean slowDrive = false;
    private boolean turretInRange;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 1
    );
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    private double ballsTrueStartTime = -1; // -1 means "not currently tracking"
    private boolean balls = false;

    private boolean zapLeon;

    private enum lightcolour { red, green, off }
    enum lightmode  { solid, blink }

    private lightcolour currentcolour = lightcolour.off;
    private lightmode  currentmode  = lightmode.solid;

    private boolean camTimerReset = false;


    private enum launchMode {
        PID,
        bang
    } private launchMode currentLaunchMode = launchMode.PID;

    private Pose adjustedGoal = new Pose(globals.turret.goalX, globals.turret.goalY);


    @Override
    public void init() {
        timer.startTime();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        t1 = new ServoEx(hardwareMap, "t1", 355);
        t2 = new ServoEx(hardwareMap, "t2", 355);
        t2.setInverted(true);
        t1.setInverted(true);

        l1 = new Motor(hardwareMap, "l1", 28, 6000);
        l2 = new Motor(hardwareMap, "l2", 28, 6000);
        l1.setRunMode(Motor.RunMode.RawPower);
        l2.setRunMode(Motor.RunMode.RawPower);
        l2.setInverted(true);
        l1.setInverted(false);
        l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        bb = hardwareMap.get(DigitalChannel.class, "bb");
        bb.setMode(DigitalChannel.Mode.INPUT);


        gate = new ServoEx(hardwareMap, "gate");


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

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        launchPIDF.setTolerance(300);


        while (timer.seconds() < 1) {
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(globals.states.autoEndPose);

        timer.reset();
        timer.startTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(85);
        limelight.pipelineSwitch(0);
        limelight.start();

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 30);

    }

    @Override
    public void loop() {
        drive();
        telemetry();
        launch();
        launchCalc();
        follower.update();
        turretAim();
        drawRobot(follower.getPose(), robotLook);
        panelsField.update();
        RPM();
        sensory();
        globals.states.autoEndPose = follower.getPose();

    }

    private void telemetry() {
        telemetry.addData("autoaim", autoAim);
        telemetry.addData("robotLocation", robotLocation);
        telemetry.addData("distance", dist);
        telemetry.addData("offset", offset);
        telemetry.addData("launch Mode", currentLaunchMode);
        TelemetryPacket ang = new TelemetryPacket();
        ang.put("target", targetRPM);

        TelemetryPacket fang = new TelemetryPacket();
        fang.put("rpm", RPM);

        FtcDashboard.getInstance().sendTelemetryPacket(ang);
        FtcDashboard.getInstance().sendTelemetryPacket(fang);




        telemetry.update();
    }

    private void launch() {

        boolean RPMDip = previousRPM - RPM > 150;
        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        boolean launchReady = (launchPIDF.atSetPoint()) && !robotLocation.equals("No Zone") && turretInRange && RPM > 500;


        if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            currentIntakeState = intakeState.launching;
        } else if (g1.getButton(GamepadKeys.Button.TRIANGLE) || g2.getButton(GamepadKeys.Button.TRIANGLE)) {
            currentIntakeState = intakeState.intaking;
        } else if (g1.getButton(GamepadKeys.Button.SQUARE)) {
            currentIntakeState = intakeState.eject;
        } else {
            currentIntakeState = intakeState.idle;

        }


        if (g2.getButton(GamepadKeys.Button.CROSS)) {


            if (RPMDip) {
                currentLaunchMode = launchMode.bang;
            }

            hood.set(MathFunctions.clamp(hoodAngle, 0, 300));
            launchPIDF.setSetPoint(targetRPM);
            launchPower = launchPIDF.calculate(RPM);

            switch (currentLaunchMode) {
                case PID:
                    l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    break;
                case bang:
                    if (RPM > targetRPM + 50) {  // small deadband
                        currentLaunchMode = launchMode.PID;
                    } else {
                        l1.set(1);
                        l2.set(1);
                    }
            }

            if (launchPIDF.atSetPoint()) {
                currentcolour = lightcolour.green;
            } else {
                currentcolour = lightcolour.red;
            }
        } else {
            currentLaunchMode = launchMode.PID;
            l1.set(0);
            l2.set(0);
            currentcolour = lightcolour.off;
        }


        switch (currentIntakeState) {

            case idle:
                intake.set(0);
                transfer.set(0);

                break;

            case launching:
                if (launchReady) {
                    gate.set(globals.gate.open);
                }
                if (turretInRange) {
                    if (robotZone.isInside(closeLaunchZone)) {
                        transfer.set(1);
                        intake.set(1);
                    } else {
                        transfer.set(0.7);
                        intake.set(0.7);
                    }
                }
                break;
            case intaking:
                transfer.set(0.75);
                intake.set(1);
                gate.set(globals.gate.close);
                break;

            case eject:
                intake.set(-0.75);
                transfer.set(-0.75);
        }
    }

    private void launchCalc() {

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Pose robot = new Pose(x, y);
        robotZone.setPosition(x, y);
        robotZone.setRotation(follower.getPose().getHeading());
        Pose goal = new Pose(6, 142);

        Pose target = goal.minus(robot);
        Vector robotToGoal = target.getAsVector();
        double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

        turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
        dist = robotToGoal.getMagnitude();


        if (robotZone.isInside(closeLaunchZone)) {

            robotLocation = "Close Zone";
            if (dist < 55) {
                targetRPM = 14 * dist + 2010;
                hoodAngle = 4.2 * dist - 159;
            } else if (dist  < 75) {
                targetRPM = -1.1429 * Math.pow(dist, 2) + 172 * dist - 3322.9;
                hoodAngle = 100;
            } else {
                targetRPM = 20 * dist + 1600;
                hoodAngle = 2 * dist -50;
            }
        } else if (robotZone.isInside(farLaunchZone)) {

            robotLocation = "Far Zone";
            if (dist < 150) {
                targetRPM = 14.433 * dist + 2064.1;
                hoodAngle = 1.9704 * dist - 124.67;
            } else {
                targetRPM = 14.286 * dist + 2185.7;
                hoodAngle = 0.7143 * dist + 44.286;
            }

        } else {
            robotLocation = "No Zone";
            if (follower.getPose().getY() > 65) {
                targetRPM = 3300;
                hoodAngle = 120;
            } else {
                targetRPM = 4300;
                hoodAngle = 120;
            }
        }


    }
    private void turretAim() {


        tagReady = false;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getStaleness() < 500) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag.getFiducialId() == 20) { //24 for red i think
                            tagAng = tag.getTargetXDegrees();
                            tagReady = true;
                        }
                    }
                }
            }
        }

        if (g2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !prevTriggerR) {
            offset -= 2;
        } else if (g2.getButton(GamepadKeys.Button.LEFT_BUMPER) && !prevTriggerL) {
            offset += 2;
        }
        if (g2.getButton(GamepadKeys.Button.DPAD_UP)) {
            offset = 0;
        }
        prevTriggerR = g2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        prevTriggerL = g2.getButton(GamepadKeys.Button.LEFT_BUMPER);

        if (autoAim) {
            if (turretInRange) {
                // only go out of range if it exceeds 155
                if (Math.abs(turretAng) > 145) {
                    turretInRange = false;
                    turretAng = 0;
                    t1.set(180);
                    t2.set(180);
                }
            } else {
                // only come back in range if it drops below 145
                if (Math.abs(turretAng) <= 140) {
                    turretInRange = true;
                } else {
                    t1.set(180);
                    t2.set(180);
                }
            }

            if (turretInRange) {
                double set = MathFunctions.clamp((177.5 - (turretAng * globals.turret.mult) + offset), 25, 335);

                if (tagReady && Math.abs(tagAng) > 0.5 && g2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    if (robotZone.isInside(farLaunchZone)) {
                        offset -= globals.turret.camP * (tagAng + globals.turret.turretOffset);
                    } else {
                        offset -= globals.turret.camP * tagAng;
                    }

                }
                t1.set(set);
                t2.set(set);
                ;
            }
        } else {
            double set = MathFunctions.clamp((180 + offset), 25, 335);
            t1.set(set);
            t2.set(set);
        }


    }

    private void sensory() {
        if (!robotLocation.equals("No Zone")) {
            currentmode = lightmode.solid;
        } else {
            currentmode = lightmode.blink;
        }
        double currentTime = timer.seconds();
        if (currentIntakeState.equals(intakeState.intaking)) {
            boolean sensorState = bb.getState();

            if (sensorState) {
                if (ballsTrueStartTime < 0) {
                    ballsTrueStartTime = currentTime;
                } else if (currentTime - ballsTrueStartTime >= 0.3) {
                    balls = true;
                }
            } else {
                ballsTrueStartTime = -1;
                balls = false;
            }
        }

        if (balls && !zapLeon) {
            gamepad1.rumble(0.6, 0.6, 200);
            zapLeon = true;
        }

        if (!balls) {
            zapLeon = false;
        }

        if (currentcolour != lightcolour.off){
            if (currentmode == lightmode.solid) {
                if (currentcolour == lightcolour.red) {
                    lights.set(0.33); // your orange position value
                } else {
                    lights.set(0.5); // your green position value
                }

            } else if (currentmode == lightmode.blink) { // BLINK
                if (timer.seconds() % 0.5 < 0.25) { // switches every 0.25s
                    if (currentcolour == lightcolour.red) {
                        lights.set(0.33); // orange
                    } else {
                        lights.set(0.5); // green
                    }
                } else {
                    lights.set(0.0); // off
                }
            }
        }else {
            lights.set(0);
        }





    }

    private void drive() {
        leftX = g1.getLeftX();
        leftY = g1.getLeftY();
        if (Math.abs(leftX) < 0.2) {
            leftX = 0;
        }
        if (Math.abs(leftY) < 0.2) {
            leftY = 0;
        }

        if (g1.getButton(GamepadKeys.Button.DPAD_UP)) {

            follower.setPose(new Pose( 8.5, 9, Math.toRadians(180)));
        }

        if (g1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            follower.setPose(new Pose(72, 9, Math.toRadians(90)));

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_DOWN)) {

            follower.setPose(new Pose(135, 9, Math.toRadians(0)));

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            follower.setPose(new Pose(15, 79, Math.toRadians(90)));

        }



        if (g1.getButton(GamepadKeys.Button.CROSS) && !prevCross1) {
            slowDrive = !slowDrive;
        } prevCross1 = g1.getButton(GamepadKeys.Button.CROSS);

        if (slowDrive) {
            follower.setMaxPower(0.6);
        } else {
            follower.setMaxPower(1);
        }

        follower.setTeleOpDrive(leftY, -leftX, 0.75 * (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), true);

    }
    public void RPM() {
        double currentTime = getRuntime();
        int currentPosition = l1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }


    public void drawRobot(Pose pose, Style style) {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double heading = follower.getHeading();
        double radius = 9.0;


        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(radius);


        Vector v = follower.getPose().getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = x + v.getXComponent() / 2, y1 = y + v.getYComponent() / 2;
        double x2 = x + v.getXComponent(), y2 = y + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        panelsField.update();
    }

}

