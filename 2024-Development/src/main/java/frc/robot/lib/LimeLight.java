package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {


    NetworkTable LIMELIGHT = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry LIMELIGHT_TX = LIMELIGHT.getEntry("tx");
    NetworkTableEntry LIMELIGHT_TY = LIMELIGHT.getEntry("ty");
    NetworkTableEntry LIMELIGHT_TA = LIMELIGHT.getEntry("ta");
    NetworkTableEntry LIMELIGHT_TV = LIMELIGHT.getEntry("tv");

    NetworkTableEntry targetpose_cameraspace = LIMELIGHT.getEntry("targetpose_cameraspace");

    NetworkTableEntry robotPose_wpiBlue = LIMELIGHT.getEntry("botpose_wpiblue");

    public PIDController strafePID;

    public LimeLight() {
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LimeLight Has Target",getLimeLightTV());
        SmartDashboard.putNumber("LimeLight X Delta", getLimeLightTX());
        SmartDashboard.putNumber("LimeLight Y Delta", getLimeLightTY());
        SmartDashboard.putNumber("LimeLight Target Area", getLimeLightTA());
        
        //SmartDashboard.putNumber("Distance Equation TEST",6+27.875/Math.sin(Math.toRadians(getLimeLightTY())));

        SmartDashboard.putNumberArray("Vision Robot Coordinates",robotPose_FieldSpace());
    
    }

    public double getLimeLightTX() {
        return LIMELIGHT_TX.getDouble(0.0);
    }

    public double getLimeLightTY() {
        return LIMELIGHT_TY.getDouble(0.0);
    }

    public double getLimeLightTA() {
        return LIMELIGHT_TA.getDouble(0.0);
    }

    public boolean getLimeLightTV() {
        return LIMELIGHT_TV.getDouble(0.0) == 1;
    }

    public double[] targetpose_cameraspace(){
        return targetpose_cameraspace.getDoubleArray(new double[0]);
    }

    public double[] robotPose_FieldSpace() {
        return robotPose_wpiBlue.getDoubleArray(new double[7]);
    }

    public Pose2d getRoboPose() {
        double[] limelightData = robotPose_FieldSpace();
        Pose2d coordinates = new Pose2d(limelightData[0],limelightData[1],new Rotation2d(limelightData[5]));
        return coordinates;
    }
    public double getRoboPoseLatency() {
        double[] limelightData = robotPose_FieldSpace();
        return limelightData[6];
    }

    public void setPipeline(double pipelineID){
        LIMELIGHT.getEntry("pipeline").setNumber(pipelineID);
    }

    public Command changePipeline(double pipelineID) {
        return Commands.runOnce(()->setPipeline(pipelineID));
    }

}
