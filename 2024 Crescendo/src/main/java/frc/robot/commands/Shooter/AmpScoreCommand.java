package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class AmpScoreCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;
    boolean previousSensorValue;
    boolean currentSensorValue;
    boolean done;
    
    public AmpScoreCommand(IntakeShooterSubsystem shooterSubsystem){

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        
        done = false;
        previousSensorValue = true;
        currentSensorValue = shooterSubsystem.getSensorValue(); 
        
        shooterSubsystem.intake(0.4);
        shooterSubsystem.runShooter(0.3, 0.3);
        
    }

    @Override
    public void execute() {

        previousSensorValue = currentSensorValue;
        currentSensorValue = shooterSubsystem.getSensorValue();

        if (currentSensorValue && !previousSensorValue){
        //if(previousSensorValue == false && currentSensorValue == true){
            done = true;
        }

        // noteDetected = shooterSubsystem.getSensorValue();
        // // if(!noteDetected){
        // //     end = (noteDetected && !detectedChange);
        // // }

        // detectedChange = noteDetected && !detectedChange;

        // if(detectedChange){
        //     end = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        new WaitCommand(1);
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}

