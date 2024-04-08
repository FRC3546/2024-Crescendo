package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SensorIntakeCommand extends Command{

    IntakeSubsystem intakeSubsystem;
    double fastSpeed;
    double slowSpeed;
    
    public SensorIntakeCommand(IntakeSubsystem intakeSubsystem, double fastSpeed, double slowSpeed){
        
        this.fastSpeed = fastSpeed;
        this.slowSpeed = slowSpeed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        intakeSubsystem.intake(fastSpeed);
    }

    @Override
    public void execute() {

        if(intakeSubsystem.getFirstSensorValue()){
            intakeSubsystem.intake(slowSpeed);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getSecondSensorValue();
    }
}

