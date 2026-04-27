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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class BLUEFAR1SPIKE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2,Path2half, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;

    private enum launchMode {
        PID,
        bang
    }  private launchMode currentLaunchMode = launchMode.PID;
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

        public void gateclose() {
            gate.set(globals.gate.close);
        }
    }

    public class OuttakeSubsystem extends SubsystemBase {
        public ServoEx turret1, turret2, hood;
        public MotorEx launcher1, launcher2;
        private final IntakeSubsystem intakeSub;
        private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
        private final PolygonZone robotZone = new PolygonZone(17, 17.5);
        public boolean tagReady = false, turretInRange, robotinZone = false, initialized = false, camTimerReset = false, launchReady = false;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        public double tagAng, turretAng, dist, offset, RPM, previousRPM, targetRPM, hoodAngle, lastTime, lastPosition, camTimer, launchPower;

        public OuttakeSubsystem(HardwareMap hardwareMap, IntakeSubsystem intakeSub) {
            this.intakeSub = intakeSub;

            turret1 = new ServoEx(hardwareMap, "t1", 360);
            turret2 = new ServoEx(hardwareMap, "t2", 360);
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
            launchReady = (launchPIDF.atSetPoint()) && robotZone.isInside(farLaunchZone) && RPM > 500 && follower.getVelocity().getMagnitude() < 2;

            if (RPMDip) {
                currentLaunchMode = launchMode.bang;
            }

            hood.set(MathFunctions.clamp(hoodAngle, 0, 300));
            launchPIDF.setSetPoint(targetRPM);
            launchPower = launchPIDF.calculate(RPM);

            double set = MathFunctions.clamp((180 - (globals.auto.farAngle * 1.054)), 25, 335);//253
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
            launcher2.set(0.7);
            launcher1.set(0.7);
            intakeSub.gateclose();
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
            Pose goal = new Pose(6, 142);

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();

            if (robotZone.isInside(farLaunchZone)) {
                launchPIDF.setTolerance(100);
                if (dist < 150) {
                    targetRPM = 14.433 * dist + 2064.1;
                    hoodAngle = 1.9704 * dist - 124.67;
                } else {
                    targetRPM = 14.286 * dist + 2185.7;
                    hoodAngle = 0.7143 * dist + 44.286;
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
                                new Pose(45.000, 9.000),

                                new Pose(globals.auto.intakex, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.intakex, 9.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(41.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(41.000, 35.000),

                                new Pose(23.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(globals.auto.intakex, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.intakex, 9.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(globals.auto.intakex, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.intakex, 9.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(globals.auto.intakex, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.intakex, 9.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(globals.auto.intakex, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.intakex, 9.000),

                                new Pose(globals.auto.shootx, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(globals.auto.shootx, 9.000),

                                new Pose(36.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

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
        follower.setStartingPose(new Pose(45, 9, Math.toRadians(180)));//TODO

        IntakeSubsystem intakeSub = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtakeSub = new OuttakeSubsystem(hardwareMap, intakeSub);
        register(intakeSub, outtakeSub);

        buildpath();

        //AUTONOMOUS ROUTINE.
        froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(3000),
                    new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path1),
                                new WaitCommand(500)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path2),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new FollowPathCommand(follower, Path3),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path4),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path5),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path6),
                                new WaitCommand(300)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path7),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path8),
                                new WaitCommand(300)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path9),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path10),
                                new WaitCommand(300)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path11),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path12),
                                new WaitCommand(300)
                        ),
                        new intakecommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path13),
                                new WaitCommand(1100)
                        ),
                        new outtakecommand(outtakeSub)
                ),
                new FollowPathCommand(follower, Path14)
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
        globals.states.autoEndPose = follower.getPose();
    }
}