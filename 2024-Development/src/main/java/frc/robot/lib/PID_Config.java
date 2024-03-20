package frc.robot.lib;

//import com.pathplanner.lib.util.PIDConstants;

public class PID_Config {
    public class SwereModule {
        //This one is really, REALLY, important all over the driving code. Try not to change it unless absolutely nessecary!
        public class ModuleTurning {
            public static final double Proportional = 0.5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class ModuleVelocity {
            //(0.001, 0.00005, 0.0005
            public static final double Proportional = 0.0001;
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
            public static final double Proportional = 0.12;
            public static final double Integral     = 0.0;
            public static final double Derivitive   = 2.0;
        }
    }

    public class ShooterSubsystem {
        public class ExtensionPID {
            public static final double Proportional = 0.5;
            public static final double Integral     = 0.0;
            public static final double Derivitive   = 0.0;
        }
        public class TilterPIDConfig {
            public class extended {
                public static final double Proportional = 0.032;//0.035;//0.0025;
                public static final double Integral     = 0.0;//0.0045;
                public static final double Derivitive   = 0.0;//0.00225;
            }
            public class home {
                public static final double Proportional = 0.022;//0.035;//0.0025;
                public static final double Integral     = 0.0;//0.0045;
                public static final double Derivitive   = 0.0;//0.00225;
            }
            public static class TilterFeedForward {
                public static final double shooterKS = 0.7;
                public static final double shooterKG = 0.32;
                public static final double shooterKV = 0.02;
                public static final double shooterKA = 0.0;
            }
        }
        public class ShooterVelocityPID {
            public static final double Proportional = 4.8839E-35;//0.0;//1.7129E-277;
            public static final double Integral     = 0.0;
            public static final double Derivitive   = 0.0;
            public class ShooterFeedForward{
                public static final double shooterKS = 0.31978;//0.45994;//0.46249;
                public static final double shooterKV = 0.13246;//0.0021643;//0.00042223;
                public static final double shooterKA = 0.025332;//0.00041488;//0.0021651;
            }
        }
    }

    public class VisionDriving {
        public class Steering {
            public static final double Proportional = 0.03;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class Strafing {                                 //TODO: Is this nessecary or could we implement the trajectory driving configs for this? Depends on method of limelight tracking.
            public static final double Proportional = 0.4;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
    }

    public class TrajectoryDriving {
        public static final double Proportional = 5;
        public static final double Integral = 0.0;
        public static final double Derivitive = 0.0;
    }
    public class TrajectoryTurning {
        public static final double Proportional = 5;
        public static final double Integral = 0.0;
        public static final double Derivitive = 0.00;
    }
    public class RotationTargetLock{
        public static final double Proportional = 0.1;
        public static final double Integral = 0.0;
        public static final double Derivitive = 0.00;
    }
}

