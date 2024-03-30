package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SensorIntakeCommand extends Command{

    IntakeSubsystem intakeSubsystem;
    double speed;
    
    public SensorIntakeCommand(IntakeSubsystem intakeSubsystem, double speed){
        
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        intakeSubsystem.intake(speed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {

        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getSecondSensorValue();
    }
}

