package frc.robot.lib;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

//pheonix
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax.ControlType;
//wpi
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Constants.ModuleConstants;
import frc.robot.lib.Constants.SwerveSubsystemConstants;

public class SwerveModule extends SubsystemBase {
    
    public final CANSparkMax MOTOR_TURN;
    public final CANSparkMax MOTOR_DRIVE;

    public final RelativeEncoder ENCODER_TURN;
    public final RelativeEncoder ENCODER_DRIVE;

    private final SparkPIDController PID_VELOCITY;

    private final PIDController PID_TURNING;

    private final CANCoder ENCODER_ABSOLUTE;
    private final double OFFSET_ABSOLUTEENCODER;

    private final String MODULE_NAME;

    private final SimpleMotorFeedforward FEEDFORWARD_VELOCITY;

    //private final SysIdRoutine m_SysIdRoutine;

    /**
    *@param    int ID_MOTOR_DRIVE,
    *@param    boolean REVERSE_MOTOR_DRIVE,
    *@param    int ID_MOTOR_TURN,
    *@param    boolean REVERSE_MOTOR_TURN,
    *@param    int ID_ENCODER_ABSOLUTE,
    *@param    boolean REVERSE_ENCODER_ABSOLUTE,
    *@param    double OFFSET_ENCODER_ABSOLUTE
    */
    public SwerveModule(
        String _NAME,
        int ID_MOTOR_DRIVE,
        boolean REVERSE_MOTOR_DRIVE,
        int ID_MOTOR_TURN,
        boolean REVERSE_MOTOR_TURN,
        int ID_ENCODER_ABSOLUTE,
        boolean REVERSE_ENCODER_ABSOLUTE,
        double OFFSET_ENCODER_ABSOLUTE
    ) {
        //init the drive motor and encoder
        MOTOR_DRIVE =new CANSparkMax(ID_MOTOR_DRIVE, MotorType.kBrushless);
        //reset to defaults
        MOTOR_DRIVE.restoreFactoryDefaults();
        //init
        MOTOR_DRIVE.setSmartCurrentLimit(60);
        MOTOR_DRIVE.setInverted(REVERSE_MOTOR_DRIVE);
        MOTOR_DRIVE.setClosedLoopRampRate(0.0001);
        ENCODER_DRIVE = MOTOR_DRIVE.getEncoder();
        ENCODER_DRIVE.setPositionConversionFactor(ModuleConstants.MODULE_DRIVE_ROTATIONS_TO_METERS);
        ENCODER_DRIVE.setVelocityConversionFactor(ModuleConstants.MODULE_DRIVE_RPM_TO_MPS);
        PID_VELOCITY = MOTOR_DRIVE.getPIDController();
        PID_VELOCITY.setP(PID_Config.SwereModule.ModuleVelocity.Proportional);
        PID_VELOCITY.setI(PID_Config.SwereModule.ModuleVelocity.Integral);
        PID_VELOCITY.setD(PID_Config.SwereModule.ModuleVelocity.Derivitive);
        //PID_VELOCITY.setOutputRange(-AutonomousConstants.LIMIT_AUTOSPEED_DRIVE, AutonomousConstants.LIMIT_AUTOSPEED_DRIVE);
        FEEDFORWARD_VELOCITY = new SimpleMotorFeedforward(
            PID_Config.SwereModule.ModuleVelocity.FeedForward.driveKS,
            PID_Config.SwereModule.ModuleVelocity.FeedForward.driveKV,
            PID_Config.SwereModule.ModuleVelocity.FeedForward.driveKA
        );
        //init the turning motor and encoder
        MOTOR_TURN = new CANSparkMax(ID_MOTOR_TURN, MotorType.kBrushless);
        //reset to defaults
        MOTOR_TURN.restoreFactoryDefaults();
        //init
        MOTOR_TURN.setSmartCurrentLimit(60);
        MOTOR_TURN.setInverted(REVERSE_MOTOR_TURN);
        this.ENCODER_TURN = MOTOR_TURN.getEncoder();
        ENCODER_TURN.setPositionConversionFactor(ModuleConstants.MODULE_TURN_ROTATIONS_TO_RADIANS);
        ENCODER_TURN.setVelocityConversionFactor(ModuleConstants.TurningEncoderRPM2RadPerSec);
        //init absolute encoder
        ENCODER_ABSOLUTE = new CANCoder(ID_ENCODER_ABSOLUTE);
        ENCODER_ABSOLUTE.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        ENCODER_ABSOLUTE.configMagnetOffset(OFFSET_ENCODER_ABSOLUTE);
        OFFSET_ABSOLUTEENCODER = OFFSET_ENCODER_ABSOLUTE;
        //init PID for turning

        this.PID_TURNING = new PIDController(PID_Config.SwereModule.ModuleTurning.Proportional,PID_Config.SwereModule.ModuleTurning.Integral,PID_Config.SwereModule.ModuleTurning.Derivitive);
        PID_TURNING.enableContinuousInput(-Math.PI, Math.PI);


        //Name
        MODULE_NAME = _NAME;

        //MOTOR_TURN.burnFlash();
        //MOTOR_DRIVE.burnFlash();

    }
    public void moduleData2Dashboard(){
        SmartDashboard.putNumber("Drive " + MODULE_NAME +" "+ ENCODER_ABSOLUTE.getDeviceID() + " angle", Math.toDegrees(getTurningPosition()));
        SmartDashboard.putNumber("Drive " + MODULE_NAME +" "+ ENCODER_ABSOLUTE.getDeviceID() + " absolute angle", Math.toDegrees(getAbsoluteEncoder()));

        //SmartDashboard.putNumber("Drive " + MODULE_NAME + " Distance Travelled",getDrivePosition());
        //SmartDashboard.putNumber("Drive " + MODULE_NAME + " Velocity", getDriveVelocity());
    }

    public boolean isIdle() {
        return
        getDriveVelocity()  <=0.1; 
        //&& getTurningVelocity()<=0.01;
    }

    /** 
     * turn encoder to absolute encoders value
    */
    public void resetTurnEncoders() {
        //ENCODER_DRIVE.setPosition(0);
        if (ENCODER_ABSOLUTE.getLastError().value == 0) {
            ENCODER_TURN.setPosition(getAbsoluteEncoder()); // Check for good data, if its okay, then reset.
        }
    }
    public void resetDriveEncoders(){
        ENCODER_DRIVE.setPosition(0.0);
    }
    /**
     * Turns SwerveModuleState into turning and driving speed
     */
    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        setAngle(state);
        if   (isOpenLoop){setPercentOutput(state);}
        else             {setSpeed(state);}
        SmartDashboard.putString(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " state", state.toString());
    }
    /**
     * Set a new angle to the turning motor
     */
    public void setAngle(SwerveModuleState state) {
        MOTOR_TURN.set(PID_TURNING.calculate(getTurningPosition(), state.angle.getRadians()));
    }
    /**
     * Set new speed for the driving motors
     */
    public void setSpeed(SwerveModuleState state) {
        PID_VELOCITY.setReference(state.speedMetersPerSecond, ControlType.kVelocity,0,FEEDFORWARD_VELOCITY.calculate(state.speedMetersPerSecond));
    }

    public void setPercentOutput(SwerveModuleState state) {
        double percentOutput = state.speedMetersPerSecond/SwerveSubsystemConstants.LIMIT_HARD_SPEED_DRIVE;
        MOTOR_DRIVE.set(/*-*/percentOutput);
    }
    
    public void stopModuleMotors(){
        MOTOR_DRIVE.stopMotor();
        MOTOR_TURN.stopMotor();
    }


    /**
     * @return drive position in meters
     */
    public double getDrivePosition(){
        return ENCODER_DRIVE.getPosition();
    }
    /**
     * @return turning position in radians
     */
    public double getTurningPosition(){
        return ENCODER_TURN.getPosition();
    }
    /**
     * @return meters per second
     */
    public double getDriveVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }
    /**
     * @return radians per second
     */
    public double getTurningVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }
    /**
     * @return absolute value in radians
     */
    public double getAbsoluteEncoder(){
        return Math.toRadians(ENCODER_ABSOLUTE.getAbsolutePosition()) + OFFSET_ABSOLUTEENCODER;
    }
    /**
     * @return swerve module state (Speed in meters per second, Angle in radians)
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
    }

    public double getModuleCurrent(){
        return MOTOR_DRIVE.getOutputCurrent() + MOTOR_TURN.getOutputCurrent();
    }
}
