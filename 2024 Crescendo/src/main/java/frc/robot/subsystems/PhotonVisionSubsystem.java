package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase{

    PhotonCamera camera = new PhotonCamera("photonvision");

    public PhotonVisionSubsystem(){}

    public double getX(){
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        return target.getYaw();
    }

    public double getY(){
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        return target.getPitch();
    }

    public boolean isTargetSpotted(){
        return camera.getLatestResult().hasTargets();
    }
}
