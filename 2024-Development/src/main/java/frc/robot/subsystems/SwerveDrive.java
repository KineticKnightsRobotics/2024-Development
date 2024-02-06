package frc.robot.subsystems;

//kauai
import com.kauailabs.navx.frc.AHRS;

//pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

//wpi
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//robot
import frc.robot.lib.Constants;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Constants.KinematicsConstants;
import frc.robot.lib.Constants.ModuleConstants;
import frc.robot.lib.Constants.SwerveSubsystemConstants;
import frc.robot.lib.PID_Config.TrajectoryDriving;
import frc.robot.lib.PID_Config.TrajectoryTurning;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


//java
import java.util.concurrent.TimeUnit;


public class SwerveDrive extends SubsystemBase {
    private Field2d field = new Field2d();

    
    private final SwerveModule MODULE_FRONT_LEFT = new SwerveModule(
        "Front Left",
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE
        ); 
    private final SwerveModule MODULE_BACK_LEFT = new SwerveModule(
        "Back Left",
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_LEFT_ENCODER_ABSOLUTE
        );
    private final SwerveModule MODULE_FRONT_RIGHT = new SwerveModule(
        "Front Right",
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE
        );  
    private final SwerveModule MODULE_BACK_RIGHT = new SwerveModule(
        "Back Right",
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE
        );  

    private final AHRS navX = new AHRS();

    private final SwerveDriveOdometry ODEMETER = new SwerveDriveOdometry(KinematicsConstants.KINEMATICS_DRIVE_CHASSIS, getRotation2d(), getModulePositions());

    public SwerveDrive() {
        try {TimeUnit.SECONDS.sleep(1);}
        catch(InterruptedException e){}
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> ODEMETER.getPoseMeters(), // Robot pose supplier
                this::resetOdometer, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(TrajectoryDriving.Proportional,TrajectoryDriving.Integral,TrajectoryDriving.Derivitive), // Translation PID constants
                        new PIDConstants(TrajectoryTurning.Proportional,TrajectoryTurning.Integral,TrajectoryTurning.Derivitive), // Rotation PID constants
                        0.05, // Max module speed, in m/s
                        KinematicsConstants.RADIUS_DRIVE_CHASSIS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
                field.setRobotPose(getPose());



    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getRobotHeading());
        SmartDashboard.putData("Field", field);


        MODULE_FRONT_LEFT.moduleData2Dashboard();
        MODULE_FRONT_RIGHT.moduleData2Dashboard();
        MODULE_BACK_LEFT.moduleData2Dashboard();
        MODULE_BACK_RIGHT.moduleData2Dashboard();

        //Ticks
        SmartDashboard.putNumber("FRONT left ticks", MODULE_FRONT_LEFT.getDrivePosition());
        SmartDashboard.putNumber("FRONT right ticks", MODULE_FRONT_RIGHT.getDrivePosition());
        SmartDashboard.putNumber("BACK left ticks", MODULE_BACK_LEFT.getDrivePosition());
        SmartDashboard.putNumber("BACK right ticks", MODULE_BACK_RIGHT.getDrivePosition());

        //RPM
        SmartDashboard.putNumber("FRONT left RPM", MODULE_FRONT_LEFT.getDriveVelocity());
        SmartDashboard.putNumber("FRONT right RPM", MODULE_FRONT_RIGHT.getDriveVelocity());
        SmartDashboard.putNumber("BACK left RPM", MODULE_BACK_LEFT.getDriveVelocity());
        SmartDashboard.putNumber("BACK right RPM", MODULE_BACK_RIGHT.getDriveVelocity());

        SmartDashboard.putNumber("BACK left Absolute Encoder", MODULE_BACK_LEFT.getAbsoluteEncoder());
        SmartDashboard.putNumber("BACK right Absolute Encoder", MODULE_BACK_RIGHT.getAbsoluteEncoder());
        SmartDashboard.putNumber("FRONT right Absolute Encoder", MODULE_FRONT_RIGHT.getAbsoluteEncoder());
        SmartDashboard.putNumber("FRONT left Absolute Encoder", MODULE_FRONT_LEFT.getAbsoluteEncoder());

        SmartDashboard.putNumber("BACK right Turning Position", MODULE_BACK_RIGHT.getTurningPosition());
        SmartDashboard.putNumber("BACK left Turning Position", MODULE_BACK_LEFT.getTurningPosition());
        SmartDashboard.putNumber("FRONT right Turning Position", MODULE_FRONT_RIGHT.getTurningPosition());
        SmartDashboard.putNumber("FRONT left Turning Position", MODULE_FRONT_LEFT.getTurningPosition());






        ODEMETER.update(
            getRotation2d(),
            getModulePositions()
        );
        
        SmartDashboard.putString("Robot Odemeter position", ODEMETER.getPoseMeters().toString());


    }

    public Pose2d getPose() {
        return ODEMETER.getPoseMeters();
      }

    public ChassisSpeeds getChassisSpeeds() {
        return KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toChassisSpeeds(getModuleStates()); //TODO:This needs to be tested!
    }

    public double getRobotHeading() {
        return Math.IEEEremainder(Constants.SwerveSubsystemConstants.REVERSED_GYRO ? -navX.getAngle() : navX.getAngle() , 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }

    public void resetOdometer(Pose2d pose){
        ODEMETER.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
    public Command resetDriveOdemeter(Pose2d pose) {
        return Commands.runOnce(()-> resetOdometer(pose));
    }

    

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = MODULE_FRONT_LEFT.getModulePosition();
        positions[1] = MODULE_FRONT_RIGHT.getModulePosition();
        positions[2] = MODULE_BACK_LEFT.getModulePosition();
        positions[3] = MODULE_BACK_RIGHT.getModulePosition();
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] newModuleStates = {
        MODULE_FRONT_LEFT.getModuleState(),
        MODULE_FRONT_RIGHT.getModuleState(),
        MODULE_BACK_LEFT.getModuleState(),
        MODULE_BACK_RIGHT.getModuleState()
        };
        return newModuleStates;
    }

    public void setChassisSpeed(ChassisSpeeds speed) {
        SwerveModuleState states[] = Constants.KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(speed);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState states[]) {
        MODULE_FRONT_LEFT.setDesiredState(states[0]);
        MODULE_FRONT_RIGHT.setDesiredState(states[1]);
        MODULE_BACK_LEFT.setDesiredState(states[2]);
        MODULE_BACK_RIGHT.setDesiredState(states[3]);
    }

    public void zeroModules(){
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();   
    }
  

    public Command zeroRobotHeading() {
        return Commands.runOnce(() -> navX.zeroYaw());
    }

    public Command zeroModuleAngles() {
        return Commands.runOnce(()-> zeroModules());
    }



    public Command followPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return new SequentialCommandGroup(
            new FollowPathHolonomic(
                path,
                () -> ODEMETER.getPoseMeters(),
                () -> getChassisSpeeds(),
                this::setChassisSpeed,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(TrajectoryDriving.Proportional,TrajectoryDriving.Integral,TrajectoryDriving.Derivitive),
                    new PIDConstants(TrajectoryDriving.Proportional,TrajectoryDriving.Integral,TrajectoryDriving.Derivitive),
                    1.0,
                    KinematicsConstants.RADIUS_DRIVE_CHASSIS,
                    new ReplanningConfig()
                ),
                () -> {
                    return false;
                },
                this
            ),
            new InstantCommand( () -> setChassisSpeed(new ChassisSpeeds(0,0,0)))
        );
    }
}
