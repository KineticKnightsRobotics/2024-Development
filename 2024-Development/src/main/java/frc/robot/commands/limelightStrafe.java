package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.LimeLight;
import frc.robot.lib.Constants;
import frc.robot.subsystems.SwerveDrive;


public class limelightStrafe extends Command {

    LimeLight   SUBSYSTEM_LIMELIGHT;
    SwerveDrive SUBSYSTEM_SWERVEDRIVE;
    double speedX,speedZ,
           targetX,targetZ,
           deltaX,deltaZ;
    DoubleSupplier zSupplier;
    ChassisSpeeds chassis_Speed;

    //TrapezoidProfile.Constraints StrafeTrapProfile = new TrapezoidProfile.Constraints(0.05,0.05);

    /**
     * Note: Positive X -> Right from camera perspective | Positive Y -> Down from camera perspective
     * @param m_subsystem1 Limelight
     * @param m_subsystem2 Drive
     * @param _targetX Meters to POI on X axis
     * @param zDoubleSupplier Meters to POI on Z axis
     */
    public limelightStrafe(LimeLight m_subsystem1, SwerveDrive m_subsystem2, double _targetX, DoubleSupplier _zSupplier){
        addRequirements(m_subsystem1,m_subsystem2);
        SUBSYSTEM_LIMELIGHT  = m_subsystem1;
        SUBSYSTEM_SWERVEDRIVE= m_subsystem2;
        targetX = _targetX;
        zSupplier = _zSupplier;

    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        //get Limelight Data
        //0: X displacement
        //1: Y displacement
        //2: Z displacement
        // Limelight Z -> Robot Y
        SmartDashboard.putData(SUBSYSTEM_LIMELIGHT.strafePID);
        double[] positionalData = SUBSYSTEM_LIMELIGHT.targetpose_cameraspace();



        deltaX = positionalData[0];

        deltaZ = positionalData[2];

        SmartDashboard.putNumber("deltaX", deltaX);
        SmartDashboard.putNumber("deltaZ", deltaZ);


        speedX = SUBSYSTEM_LIMELIGHT.strafePID.calculate(-deltaX, targetX);
        SmartDashboard.putNumber("speed X", speedX);

        /*
        if (zStrafe){speedZ = SUBSYSTEM_LIMELIGHT.strafePID.calculate(deltaZ, targetZ);}
        else        {speedZ = 0;}
        SmartDashboard.putNumber("speed Y", speedZ);
        */

        double zSpeed = zSupplier.getAsDouble() * Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * 0.2 * (Math.abs(zSupplier.getAsDouble()) > 0.1 ? 1.0 : 0.0);


        chassis_Speed = new ChassisSpeeds(speedX,zSpeed,0.0);

        SUBSYSTEM_SWERVEDRIVE.setAutoChassisSpeed(chassis_Speed);



    }
    @Override
    public void end(boolean interrupted){
        chassis_Speed = new ChassisSpeeds(0,0,0);SUBSYSTEM_SWERVEDRIVE.setAutoChassisSpeed(chassis_Speed);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    
}