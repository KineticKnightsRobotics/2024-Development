package frc.robot.lib;

import com.pathplanner.lib.util.PIDConstants;

public class PID_Config {
    public class SwereModule {
        public class ModuleTurning {
            public static final double Proportional = 0.5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class ModuleVelocity {
            //(0.001, 0.00005, 0.0005
            public static final double Proportional = 0.001;
            public static final double Integral = 0.00005;
            public static final double Derivitive = 0.0005;
            public class FeedForward{
                public static final double driveKS = 0.667;
                public static final double driveKV = 2.44;
                public static final double driveKA = 0.27;
            }
        }
    }

    public class IntakeSubsystem {
        public class IntakePivotControllerPID {
            public static final double Proportional = 0.03;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
    }

    public class ShooterSubsystem {
        public class TilterPIDConfig {
            public static final double Proportional = 0.5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class ShooterVelocityPID {
            public static final double Proportional = 0.5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }


        //Currently unused
        public class FeederPositionKinematicsPID {
            public static final double Proportional = 0.1;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
    }

    public class VisionDriving {
        public class Steering {
            public static final double Proportional = 0.03;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class Strafing {
            public static final double Proportional = 0.4;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
    }

    public class TrajectoryDriving {

    //public static final PIDConstants translationPID = new PIDConstants(3, 0.5, 0);
    //public static final PIDConstants rotationPID = new PIDConstants(2, 0.002, 0.05);
        public static final double Proportional = 5.0;
        public static final double Integral = 0.0;
        public static final double Derivitive = 0.0;
    }
    public class TrajectoryTurning {
        public static final double Proportional = 2.0;
        public static final double Integral = 0.002;
        public static final double Derivitive = 0.005;
}

}

