package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.geometry.Translation2d;

//import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveDrive;

public class joystickDrive extends Command {

    private final SwerveDrive subsystem;
    private final DoubleSupplier SUPPLIER_xSpeed;
    private final DoubleSupplier SUPPLIER_ySpeed;
    private final DoubleSupplier SUPPLIER_zSpeed;
    private final BooleanSupplier SUPPLIER_Field_Oriented;
    private final BooleanSupplier flipControls;
    private final DoubleSupplier SUPPLIER_Period;

    //private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public joystickDrive(
        SwerveDrive m_subsystem,
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed, 
        DoubleSupplier zSpeed,
        BooleanSupplier fieldOriented,
        BooleanSupplier _flipControls,
        DoubleSupplier timePeriod
        ){
        subsystem = m_subsystem;
        SUPPLIER_xSpeed = xSpeed;
        SUPPLIER_ySpeed = ySpeed;
        SUPPLIER_zSpeed = zSpeed;
        SUPPLIER_Field_Oriented = fieldOriented;
        flipControls = _flipControls;
        SUPPLIER_Period = timePeriod;
        addRequirements(subsystem);
        //this.xLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.yLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.zLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_TURN);
    }

    @Override
    public void execute() {
        double joystickX = SUPPLIER_xSpeed.getAsDouble();
        double joystickY = SUPPLIER_ySpeed.getAsDouble(); //grab speeds and apply deadband
        double joystickZ = SUPPLIER_zSpeed.getAsDouble();

        if (flipControls.getAsBoolean()) {
            joystickX *= -1;
            joystickY *= -1;
            //joystickZ *= -1;
        }

        //double xSpeed   = (Math.pow(joystickX, 2) * (joystickX<0 ? -1 : 1) /1.0) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * (RobotContainer.DRIVER_LT() ? 0.3 : 1);      // * 0.2;
        //double ySpeed   = (Math.pow(joystickY, 2) * (joystickY<0 ? -1 : 1) /1.0) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * (RobotContainer.DRIVER_LT() ? 0.3 : 1);      // * 0.2; //Determine new velocity
        //double rotSpeed = (Math.pow(joystickZ, 2) * (joystickZ<0 ? -1 : 1) /1.0)*0.8*SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN  * (RobotContainer.DRIVER_LT() ? 0.3 : 1); //* 0.8 to make SLOW */
        
        double xSpeed   = ((joystickX * joystickX) * (joystickX<0 ? -1 : 1)) *    SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;// * 0.2;
        double ySpeed   = ((joystickY * joystickY) * (joystickY<0 ? -1 : 1)) *    SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;
        double rotSpeed = ((joystickZ * joystickZ) * (joystickZ<0 ? -1 : 1)) *0.8*SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN; //* 0.8 to make SLOW */


        //apply slow mode
        if (RobotContainer.DRIVER_LT()) {
            xSpeed   *= 0.3;
            ySpeed   *= 0.3;
            rotSpeed *= 0.3;
        }


        
        boolean fieldRelative = SUPPLIER_Field_Oriented.getAsBoolean();
        double timePeriod = SUPPLIER_Period.getAsDouble();

        //ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d());
      
        ChassisSpeeds chassisSpeed = ChassisSpeeds.discretize(
            fieldRelative 
                ? 
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d())
                : 
                    new ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
            timePeriod);
     
        
        subsystem.setChassisSpeed(chassisSpeed,true);
        SmartDashboard.putNumber("RT", RobotContainer.DRIVER_RT());
        /*
        if (subsystem.idle_Timer_Lock.get() > 0.5) {
            subsystem.lockChassis();
            subsystem.idle_Timer_Lock.reset();
        }
        */
    }
    @Override
    public void end(boolean interrupted) {
        //this.swerve.drive(new Translation2d(0, 0), 0, true, false);
        //PLEASE SET THIS FOR SAFETY!!!
        //this.swerve.stopMotors();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
