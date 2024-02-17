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
            public static final double Integral = 0.0;
            public static final double Derivitive = 2.0;
        }
    }

    public class ShooterSubsystem {
        public class TilterPIDConfig {
            public static final double Proportional = 0.1;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
        public class ShooterVelocityPID {
            public static final double Proportional = 6e-5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
            public class ShooterFeedForward{
                public static final double shooterKV = 0.38;
                public static final double shooterKA = 0.20;
            }
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
}

