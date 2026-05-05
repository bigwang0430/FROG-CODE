package org.firstinspires.ftc.teamcode.AUTO;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.TELEOP.Blue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Objects;

@Autonomous
public class REDCLOSE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;

    private enum launchMode {
        PID,
        bang
    }

    private launchMode currentLaunchMode = launchMode.PID;

    //SUBSYSTEMS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public class IntakeSubsystem extends SubsystemBase {
        public final MotorEx intake, transfer;
        public final ServoEx gate;

        public IntakeSubsystem(HardwareMap hardwareMap) {


            intake = new MotorEx(hardwareMap, "intake");
            intake.stopAndResetEncoder();
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setInverted(false);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            intake.resetEncoder();

            transfer = new MotorEx(hardwareMap, "transfer");
            transfer.setRunMode(Motor.RunMode.RawPower);
            transfer.setInverted(true);
            transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            gate = new ServoEx(hardwareMap, "gate");
            gate.set(globals.gate.close);
        }


        public void startIntake() {
            intake.set(0.75);
            transfer.set(1);
            gate.set(globals.gate.close);
        }

        public void stopIntake() {
            intake.set(0);
            transfer.set(0);
            gate.set(globals.gate.close);
        }

        public void feedLauncher() {
            gate.set(globals.gate.open);
            intake.set(1);
            transfer.set(1);
        }
    }

    public class OuttakeSubsystem extends SubsystemBase {
        public ServoEx turret1, turret2, hood;
        public MotorEx launcher1, launcher2;
        private final IntakeSubsystem intakeSub;
        private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
        private final PolygonZone robotZone = new PolygonZone(17, 17.5);
        public boolean tagReady = false, turretInRange, robotinZone = false, initialized = false, camTimerReset = false, launchReady = false;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        public double tagAng, turretAng, dist, offset, RPM, previousRPM, targetRPM, hoodAngle, lastTime, lastPosition, camTimer, launchPower;

        public OuttakeSubsystem(HardwareMap hardwareMap, IntakeSubsystem intakeSub) {
            this.intakeSub = intakeSub;

            turret1 = new ServoEx(hardwareMap, "t1", 355);
            turret2 = new ServoEx(hardwareMap, "t2", 355);
            turret2.setInverted(true);
            turret1.setInverted(true);

            launcher1 = new MotorEx(hardwareMap, "l1", 28, 6000);
            launcher2 = new MotorEx(hardwareMap, "l2", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher2.setInverted(true);
            launcher1.setInverted(false);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);

            hood = new ServoEx(hardwareMap, "hood", 300);
        }


        private void launch() {
            boolean RPMDip = previousRPM - RPM > 150;
            launchReady = (launchPIDF.atSetPoint()) && robotZone.isInside(closeLaunchZone) && RPM > 500 && follower.getVelocity().getMagnitude() < 2;

            if (RPMDip) {
                currentLaunchMode = launchMode.bang;
            }

            hood.set(MathFunctions.clamp(hoodAngle, 0, 300));
            launchPIDF.setSetPoint(targetRPM);
            launchPower = launchPIDF.calculate(RPM);

            double set = MathFunctions.clamp((177.5 - (turretAng * globals.turret.mult)), 25, 335);
            turret1.set(set);
            turret2.set(set);

            switch (currentLaunchMode) {
                case PID:
                    launcher1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    launcher2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    break;
                case bang:
                    if (RPM > targetRPM + 50) {  // small deadband
                        currentLaunchMode = launchMode.PID;
                    } else {
                        launcher1.set(1);
                        launcher2.set(1);
                    }
            }

            if (launchReady){
                intakeSub.feedLauncher();
            }
        }

        public void launcheroff(){
            currentLaunchMode = launchMode.PID;
            launcher2.set(0.37);
            launcher1.set(0.37);
        }

        public void RPM() {
            double currentTime = getRuntime();
            int currentPosition = launcher1.getCurrentPosition();

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

        private void launchCalc() {
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            robotZone.setPosition(x, y);
            robotZone.setRotation(follower.getPose().getHeading());
            Pose goal = new Pose(142 - globals.turret.goalX, globals.turret.goalY);

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();

            if (robotZone.isInside(closeLaunchZone)) {
                launchPIDF.setTolerance(230);
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
            } else {
                launchPIDF.setTolerance(230);
                if (follower.getPose().getY() < 56) {
                    targetRPM = 3300;
                    hoodAngle = 120;
                } else {
                    targetRPM = 4300;
                    hoodAngle = 120;
                }
            }

        }

        @Override
        public void periodic() {
            RPM();
            launchCalc();
        }
    }

    //COMMANDS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static class intakecommand extends  CommandBase {
        private final IntakeSubsystem intakeSubsystem;

        public intakecommand(IntakeSubsystem intakeSubsystem){
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize(){
            intakeSubsystem.startIntake();
        }

        @Override
        public void end(boolean interrupted){
            intakeSubsystem.stopIntake();
        }
    }

    public static class outtakecommand extends  CommandBase {
        private final OuttakeSubsystem outtakeSubsystem;

        public outtakecommand(OuttakeSubsystem outtakeSubsystem){
            this.outtakeSubsystem = outtakeSubsystem;
            addRequirements(outtakeSubsystem);
        }

        @Override
        public void execute(){
            outtakeSubsystem.launch();
        }

        @Override
        public void end(boolean interrupted){
            outtakeSubsystem.launcheroff();
            outtakeSubsystem.intakeSub.stopIntake();
        }
    }

    //PATHS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void buildpath(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(29.000, 128.000).mirror(),

                                new Pose(50.000, 90.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 90.000).mirror(),

                                new Pose(42.000, 60.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 60.000).mirror(),

                                new Pose(23.000, 60.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 60.000).mirror(),

                                new Pose(55.000, 77.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 77.000).mirror(),

                                new Pose(12.500, 59.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(30))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 59.000).mirror(),

                                new Pose(55.000, 77.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(30))

                .build();
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 77.000).mirror(),

                                new Pose(12.500, 59.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(30))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 59.000).mirror(),

                                new Pose(55.000, 77.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(30))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 77.000).mirror(),

                                new Pose(42.000, 84.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(42.000, 84.000).mirror(),

                                new Pose(23.000, 84.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 84.000).mirror(),

                                new Pose(45.000, 84.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 84.000).mirror(),

                                new Pose(42.000, 35.400).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 35.400).mirror(),

                                new Pose(23.000, 35.400).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.400).mirror(),

                                new Pose(58.000, 110.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }

    @Override
    public void initialize() {
        //PINPOINT INITIALIZATION, CALIBRATES OUR HARDWARE BEFORE RUNNING
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        //SET INITIAL POSITION AFTER HARDWARE CALIBRATION
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(29, 128, Math.toRadians(180)).mirror()
        );//TODO 129

        IntakeSubsystem intakeSub = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtakeSub = new OuttakeSubsystem(hardwareMap, intakeSub);
        register(intakeSub, outtakeSub);

        buildpath();

        //AUTONOMOUS ROUTINE.
        froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path1),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new FollowPathCommand(follower, Path2, false),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path3),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path4),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path5),
                                new WaitCommand(1200)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path6),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path7),
                                new WaitCommand(1500)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path8),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path9),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path10),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new FollowPathCommand(follower, Path11),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path12),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path13),
                                new WaitCommand(650)
                        ),
                        new outtakecommand(outtakeSub)
                )
        );
    }

    @Override
    public void run(){
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        super.run();
        follower.update();
        globals.states.autoEndPose = new Pose(58.000, 110.000, Math.toRadians(180)).mirror();
    }
}