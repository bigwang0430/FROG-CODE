package org.firstinspires.ftc.teamcode.GLOBALS;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


@Config
public class globals {

    @Config
    public static class lift {
        public static float rightIdle = 0.085F;
        public static float rightBrake = 0.15F;
        public static float rightLift = 0.22F;
        public static float leftIdle = 0.085F;
        public static float leftBrake = 0.15F;
        public static float leftLift = 0.22F;
            }


    @Config
    public static class launcher {


        public static float p =0.003F; //0.001
        public static float i = 0.1F;
        public static float d = 0F;
        public static float ks = 0.067F; //0.0000216
        public static float kv = 0.00017F; //0.000000120871
    }

    @Config
    public static class tuning {

        public static float ang = 0F;
        public static float targetRPM = 3000F;

    }

    @Config
    public static class gate {
        public static float close = 0.25F; //
        public static float open = 0.8F;
    }

    @Config
    public static class turret {
        public static float mult = 1.14F;
        public static float camP = 0.05F;
        public static float turretOffset = 0F;
        public static float goalY = 138F;
        public static float goalX = 6;
        public static float turretangle = 302F;

    }

    @Config
    public static class states{
        public static Pose autoEndPose = new Pose(60, 84, Math.PI/2);
    }
    @Config
    public static class auto {
        public static float farAngle = 69F;
        public static float shootx = 43F;
        public static float intakex = 9F;
    }


}

