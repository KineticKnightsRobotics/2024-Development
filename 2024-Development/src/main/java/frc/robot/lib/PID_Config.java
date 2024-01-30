package frc.robot.lib;

public class PID_Config {
    public class SwereModule {
        public class ModuleTurning {
            public static final double Proportional = 0.5;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
        }
    }

    public class IntakeSubsystem {
        public class SchwoopControllerPID {
            public static final double Proportional = 0.1;
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
        public class FlywheelVelocityPID {
            public static final double Proportional = 0.5;
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
            public static final double Proportional = 0.10;
            public static final double Integral = 0.0;
            public static final double Derivitive = 0.0;
    }
}

