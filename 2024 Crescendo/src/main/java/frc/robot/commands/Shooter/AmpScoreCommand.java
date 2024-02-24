package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AmpScoreCommand extends Command{

    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    boolean previousSensorValue;
    boolean currentSensorValue;
    boolean done;
    
    public AmpScoreCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem){

        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        
        done = false;
        previousSensorValue = true;
        currentSensorValue = intakeSubsystem.getSensorValue(); 
        
        intakeSubsystem.intake(0.4);
        shooterSubsystem.runShooter(0.3, 0.3);
        
    }

    @Override
    public void execute() {

        previousSensorValue = currentSensorValue;
        currentSensorValue = intakeSubsystem.getSensorValue();

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
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}

