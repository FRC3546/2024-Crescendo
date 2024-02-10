package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax leftClimbMotor;
    private CANSparkMax rightClimbMotor;

    public ClimbSubsystem() {

        leftClimbMotor = new CANSparkMax(41, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(40, MotorType.kBrushless);

        leftClimbMotor.follow(rightClimbMotor);

    }

    public void rotateClimbers(double value) {

        rightClimbMotor.set(value);

    }

    public void stopClimber(){

        rightClimbMotor.stopMotor();
    }

}