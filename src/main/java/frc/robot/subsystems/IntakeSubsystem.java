package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.PortConstants;


public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intakeMotor1;
    private TalonFX intakeMotor2;
    private double testSpeed;

    public IntakeSubsystem(){
        intakeMotor1 = new TalonFX(PortConstants.intakeMotorPort);
        intakeMotor2 = new TalonFX(PortConstants.intakeMotorPort);
        intakeMotor1.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor2.setNeutralMode(NeutralModeValue.Brake);
        testSpeed = 0.2;



    }
    public void power(double power){
        intakeMotor1.set(power);
        intakeMotor2.set(power);
    }
    public void setMotorTestSpeed(){
        intakeMotor1.set(testSpeed);
        intakeMotor2.set(testSpeed);
    }
    public void setMotorTestSpeedNeg(){
        intakeMotor1.set(-testSpeed);
        intakeMotor2.set(-testSpeed);
    }
    public void upSpeed(){
        testSpeed += 0.05;
    }
    public void downSpeed(){
        testSpeed -= 0.05;
    }
    public void testSpeedShutdown(){
        testSpeed = 0;
    }
    public void shutdown(){
    }
}
