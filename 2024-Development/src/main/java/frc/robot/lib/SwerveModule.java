package frc.robot.lib;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//pheonix
import com.ctre.phoenix.sensors.CANCoder;
//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax.ControlType;
//wpi
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    
    private final CANSparkMax MOTOR_TURN;
    private final CANSparkMax MOTOR_DRIVE;

    private final RelativeEncoder ENCODER_TURN;
    private final RelativeEncoder ENCODER_DRIVE;

    private final PIDController PID_TURNING;

    private final CANCoder ENCODER_ABSOLUTE;
    private final double OFFSET_ABSOLUTEENCODER;

    private final String MODULE_NAME;

    /**
    *@param int ID_MOTOR_DRIVE,
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
        this.MOTOR_DRIVE =new CANSparkMax(ID_MOTOR_DRIVE, MotorType.kBrushless);
        //reset to defaults
        this.MOTOR_DRIVE.restoreFactoryDefaults();
        //init
        MOTOR_DRIVE.setInverted(REVERSE_MOTOR_DRIVE);
        MOTOR_DRIVE.setClosedLoopRampRate(2);
        this.ENCODER_DRIVE = MOTOR_DRIVE.getEncoder();
        ENCODER_DRIVE.setPositionConversionFactor(ModuleConstants.MODULE_DRIVE_ROTATIONS_TO_METERS);
        //init the turning motor and encoder
        this.MOTOR_TURN = new CANSparkMax(ID_MOTOR_TURN, MotorType.kBrushless);
        //reset to defaults
        this.MOTOR_TURN.restoreFactoryDefaults();
        //init
        MOTOR_TURN.setInverted(REVERSE_MOTOR_TURN);
        MOTOR_TURN.setClosedLoopRampRate(3); //TODO: this might mess up the PID when tested on test bot!
        this.ENCODER_TURN = MOTOR_TURN.getEncoder();
        ENCODER_TURN.setPositionConversionFactor(ModuleConstants.MODULE_TURN_ROTATIONS_TO_RADIANS);
        //init absolute encoder
        this.ENCODER_ABSOLUTE = new CANCoder(ID_ENCODER_ABSOLUTE);
        ENCODER_ABSOLUTE.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        ENCODER_ABSOLUTE.configMagnetOffset(OFFSET_ENCODER_ABSOLUTE);
        this.OFFSET_ABSOLUTEENCODER = OFFSET_ENCODER_ABSOLUTE;
        //init PID for turning
        this.PID_TURNING = new PIDController(PID_Config.SwereModule.ModuleTurning.Proportional,PID_Config.SwereModule.ModuleTurning.Integral,PID_Config.SwereModule.ModuleTurning.Derivitive);
        PID_TURNING.enableContinuousInput(-Math.PI, Math.PI);


        //Name
        MODULE_NAME = _NAME;
    }
    public void moduleData2Dashboard(){
        SmartDashboard.putNumber(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " angle", Math.toDegrees(getTurningPosition()));
        SmartDashboard.putNumber(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " absolute angle", Math.toDegrees(getAbsoluteEncoder()));
    }

    /** 
     * Resets drive encoder to 0, and turn encoder to absolute encoders value
    */
    public void resetEncoders() {
        ENCODER_DRIVE.setPosition(0);
        if (ENCODER_ABSOLUTE.getLastError().value == 0) {
            ENCODER_TURN.setPosition(getAbsoluteEncoder()); // Check for good data, if its okay, then reset.
        }
    }
    /**
     * Turns SwerveModuleState into turning and driving speed
     */
    public void setDesiredState(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state, getModuleState().angle);

        setAngle(state);
        setSpeed(state);

        SmartDashboard.putNumber(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " angle", Math.toDegrees(getTurningPosition()));
        SmartDashboard.putNumber(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " absolute angle", Math.toDegrees(getAbsoluteEncoder()));
        SmartDashboard.putString(MODULE_NAME + ENCODER_ABSOLUTE.getDeviceID() + " state", state.toString());

        //SmartDashboard.putNumber("Swerve[" + ENCODER_ABSOLUTE.getDeviceID() + "] offset", OFFSET_ABSOLUTEENCODER);
    }
    /**
     * Set a new angle to the turning motor
     */
    public void setAngle(SwerveModuleState state) {

        double currentAngle = getTurningPosition();
        double delta = PID_TURNING.calculate(currentAngle, state.angle.getRadians());

        MOTOR_TURN.set(delta);
        //MOTOR_TURN.set(delta < 0.01 ? 0.0 : delta); // if delta is less than 1% output, just stop the motor so it doesn't jitter
    }
    /**
     * Set new speed for the driving motors
     */
    public void setSpeed(SwerveModuleState state) {

        //the speed limit is full power 
        MOTOR_DRIVE.set(state.speedMetersPerSecond/Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE);
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

}
