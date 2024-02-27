package frc.robot.subsystems;

//kauai
import com.kauailabs.navx.frc.AHRS;
//pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//wpi
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
//robot
import frc.robot.lib.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Constants.KinematicsConstants;
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
    
    public final Timer idle_Timer_Zero = new Timer();
    public final Timer idle_Timer_Lock = new Timer();

    //private final SwerveDriveOdometry ODEMETER = new SwerveDriveOdometry(KinematicsConstants.KINEMATICS_DRIVE_CHASSIS, getRotation2d(), getModulePositions());
    public final SwerveDrivePoseEstimator ODEMETER = new SwerveDrivePoseEstimator(
        KinematicsConstants.KINEMATICS_DRIVE_CHASSIS,
        getRotation2d(),
        getModulePositions(),
        new Pose2d(0,0,new Rotation2d(0))
    );

    public SwerveDrive() {
        try {TimeUnit.SECONDS.sleep(1);}
        catch(InterruptedException e){}
        MODULE_FRONT_LEFT.resetTurnEncoders();
        MODULE_FRONT_RIGHT.resetTurnEncoders();
        MODULE_BACK_LEFT.resetTurnEncoders();
        MODULE_BACK_RIGHT.resetTurnEncoders();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> ODEMETER.getEstimatedPosition(), // Robot pose supplier
                this::resetOdometer, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setAutoChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(TrajectoryDriving.Proportional,TrajectoryDriving.Integral,TrajectoryDriving.Derivitive),
                    new PIDConstants(TrajectoryTurning.Proportional,TrajectoryTurning.Integral,TrajectoryTurning.Derivitive),
                    3.4,
                    KinematicsConstants.RADIUS_DRIVE_CHASSIS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(
                            
                        ) // Default path replanning config. See the API for the options here
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
        idle_Timer_Lock.reset();
        idle_Timer_Lock.start();
        idle_Timer_Zero.reset();
        idle_Timer_Zero.start();

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
                field.setRobotPose(getPose());


    }

    @Override
    public void periodic() {
       // Shuffleboard.getTab("Teleoperated").add("Gyro",navX.getYaw());
       // SmartDashboard.putNumber("Robot Heading", getRobotHeading());
        SmartDashboard.putData("Field", field);field.setRobotPose(getPose());
        SmartDashboard.putString("Robot Odemeter position", ODEMETER.getEstimatedPosition().toString());

        MODULE_FRONT_LEFT.moduleData2Dashboard();
        MODULE_FRONT_RIGHT.moduleData2Dashboard();
        MODULE_BACK_LEFT.moduleData2Dashboard();
        MODULE_BACK_RIGHT.moduleData2Dashboard();

        SmartDashboard.putNumber("Gyro Heading", -navX.getYaw());

        updatePoseEstimator();


        if (!getChassisIdle()) {
            idle_Timer_Lock.reset();
            idle_Timer_Zero.reset();
        }
        if(idle_Timer_Zero.get() > 0.1 ) {
            zeroModuleAngles();
            idle_Timer_Zero.reset();
        }

        SmartDashboard.putNumber("Idle Timer Locking", idle_Timer_Lock.get());

    }

    public Pose2d getPose() {
        return ODEMETER.getEstimatedPosition();

      }

    public void updatePoseEstimator() {
        ODEMETER.update(
            getRotation2d(),
            getModulePositions()
        );
        /* 
        if (m_LimeLight.getLimeLightTV()) {
            ODEMETER.addVisionMeasurement(
                m_LimeLight.getRoboPose(),
                Timer.getFPGATimestamp() - (m_LimeLight.getRoboPoseLatency()/1000)
            );
        }
        */
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toChassisSpeeds(getModuleStates());
    }

    public boolean getChassisIdle() {
        return
        MODULE_FRONT_LEFT.checkIdle() &&
        MODULE_BACK_LEFT.checkIdle() &&
        MODULE_FRONT_RIGHT.checkIdle() &&
        MODULE_BACK_RIGHT.checkIdle();
    }

    public double getRobotHeading() {
        return Math.IEEEremainder(Constants.SwerveSubsystemConstants.REVERSED_GYRO ? navX.getAngle() : -navX.getAngle() , 360);
        //return navX.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
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

    public void setChassisSpeed(ChassisSpeeds speed,boolean isOpenLoop) {
        SwerveModuleState states[] = Constants.KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(speed);
        setModuleStates(states,true);
    }

    public void setAutoChassisSpeed(ChassisSpeeds speed) {
        SwerveModuleState states[] = Constants.KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(speed);
        setModuleStates(states,false);
    }

    public void setModuleStates(SwerveModuleState states[], boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE);
        MODULE_FRONT_LEFT.setDesiredState(states[0],isOpenLoop);
        MODULE_FRONT_RIGHT.setDesiredState(states[1],isOpenLoop);
        MODULE_BACK_LEFT.setDesiredState(states[2],isOpenLoop);
        MODULE_BACK_RIGHT.setDesiredState(states[3],isOpenLoop);
    }

    public void zeroTurnEncoders(){
        MODULE_FRONT_LEFT.resetTurnEncoders();
        MODULE_FRONT_RIGHT.resetTurnEncoders();
        MODULE_BACK_LEFT.resetTurnEncoders();
        MODULE_BACK_RIGHT.resetTurnEncoders();   
    }

    public void zerDriveEncoders() {
        MODULE_FRONT_LEFT.resetDriveEncoders();
        MODULE_FRONT_RIGHT.resetDriveEncoders();
        MODULE_BACK_LEFT.resetDriveEncoders();
        MODULE_BACK_RIGHT.resetDriveEncoders();  
    }
  
    public void resetOdometer(Pose2d pose){
        ODEMETER.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public Command resetDriveOdemeter(Pose2d pose) {
        return Commands.runOnce(()-> resetOdometer(pose));
    }

    public Command zeroRobotHeading() {
        return Commands.runOnce(() -> navX.zeroYaw());
    }

    public Command zeroModuleAngles() {
        return Commands.runOnce(()-> zeroTurnEncoders());
    }

    public Command completeModuleZero(){
        return Commands.runOnce( ()-> {
        MODULE_FRONT_LEFT.resetDriveEncoders();
        MODULE_FRONT_RIGHT.resetDriveEncoders();
        MODULE_BACK_LEFT.resetDriveEncoders();
        MODULE_BACK_RIGHT.resetDriveEncoders();
        MODULE_FRONT_LEFT.resetTurnEncoders();
        MODULE_FRONT_RIGHT.resetTurnEncoders();
        MODULE_BACK_LEFT.resetTurnEncoders();
        MODULE_BACK_RIGHT.resetTurnEncoders();   
        });
    }

    public void lockChassis() {
        MODULE_FRONT_LEFT.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Units.degreesToRadians(45))),true);
        MODULE_BACK_LEFT.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Units.degreesToRadians(-45))),true);
        MODULE_FRONT_RIGHT.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Units.degreesToRadians(-45))),true);
        MODULE_BACK_RIGHT.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Units.degreesToRadians(45))),true);
    }
    
  /*  public Command lockDrive() {
        return Commands.runOnce(() -> lockChassis(),this);
    }*/

    

    /*
    public Command followPath(String pathName,Boolean isChoreo) {
        PathPlannerPath path;
        if ( ! isChoreo){
            path = PathPlannerPath.fromPathFile(pathName);
        }
        else {
            path = PathPlannerPath.fromChoreoTrajectory(pathName);
        }
        return new SequentialCommandGroup(
            new FollowPathHolonomic(
                path,
                () -> ODEMETER.getEstimatedPosition(),
                () -> getChassisSpeeds(),
                this::setAutoChassisSpeed, 
                new HolonomicPathFollowerConfig(
                    new PIDConstants(TrajectoryDriving.Proportional,TrajectoryDriving.Integral,TrajectoryDriving.Derivitive),
                    new PIDConstants(TrajectoryTurning.Proportional,TrajectoryTurning.Integral,TrajectoryTurning.Derivitive),
                    2,
                    KinematicsConstants.RADIUS_DRIVE_CHASSIS,
                    new ReplanningConfig()
                ),
                () -> {
                    return false;
                },
                this
            ),
            new InstantCommand( () -> setChassisSpeed(new ChassisSpeeds(0,0,0),true))
        );
    }
    */
}
