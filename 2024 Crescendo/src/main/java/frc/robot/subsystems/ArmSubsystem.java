package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class ArmSubsystem extends SubsystemBase {

    private boolean isExtended;
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;
    private DoubleSolenoid extendSolenoid;

    public ArmSubsystem() {

        leftArmMotor = new CANSparkMax(28, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(29, MotorType.kBrushless);

        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.follow(rightArmMotor, true);
        rightArmMotor.setInverted(true);
        throughBoreEncoder = rightArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        leftArmMotor.setSmartCurrentLimit(50);
        rightArmMotor.setSmartCurrentLimit(50);

        leftArmMotor.burnFlash();
        rightArmMotor.burnFlash();

        extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    }

    public void rotateArmUp() {

        leftArmMotor.set(1);
    }

    public void rotateArmDown() {

        leftArmMotor.set(-1);
    }

    public void StopArm() {

        rightArmMotor.stopMotor();
    }

    public void rotateArm(double value) {
        // negative value moves the arm down
        // postive value moves the arm up
        rightArmMotor.set(value);
    }

    public void retractArm() {

        isExtended = false;
        extendSolenoid.set(Value.kForward);
    }

    public void extendArm() {

        isExtended = true;
        extendSolenoid.set(Value.kReverse);
    }

    public void toggleArm() {
        isExtended = !isExtended;
        extendSolenoid.toggle();
    }

    public boolean getArmExtension() {
        return isExtended;
    }

    public double getArmPosition() {
        return throughBoreEncoder.getPosition();
    }

    @Override
    public void periodic() {}
}
