package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {

  public void setPipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  public double getLimelightY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getLimelightX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public boolean isLimelightXRange() {
    double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return (x > -20 && x < 20);
  }


   /**
   * Gets the pose of the bot from the Limelight. 
   * Only use late game because of alliance selection delay.
   */
  public double[] getBotPose() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double[] botPose;
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
      botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    } else {
      botPose = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }
    return botPose;
  }

  public double getTagId(){
    return  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

  }
}
