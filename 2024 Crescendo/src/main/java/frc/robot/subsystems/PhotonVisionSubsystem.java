package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase{

    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public PhotonVisionSubsystem(){}

    public double getX(){
        var result = camera.getLatestResult();
        if(result != null){
            PhotonTrackedTarget target = result.getBestTarget();
            if(target != null){
                // -9 because camera is not center
                return target.getYaw() - 9;
            }
            else{
                return 0;
            }
            
        }
        else{
            return 0;
        }
        // return 0;
    }

    public double getY(){
        var result = camera.getLatestResult();
        if(result != null){
            PhotonTrackedTarget target = result.getBestTarget();
            if(target != null){
                return target.getPitch();
            }
            else{
                return 0;
            }
        }
        else{
            return 0;
        }
    }

    public boolean isTargetSpotted(){
        var result = camera.getLatestResult();
        if(result != null){
            return result.hasTargets();
        }
        else{
            return false;
        }
    }
}
