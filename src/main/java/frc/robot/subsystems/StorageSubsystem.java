package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.PortConstants;

public class StorageSubsystem extends SubsystemBase {
    private TalonFX storageMotor;
    private double testSpeed;
    
    public StorageSubsystem(){
        storageMotor = new TalonFX(PortConstants.StorageMotorPort);
        storageMotor.setNeutralMode(NeutralModeValue.Brake);
        testSpeed= 0.2;
    }
    public void setPower(double power){
        storageMotor.set(power);
    }
    public void setMotorTestSpeed() {
        storageMotor.set(testSpeed);
    }
    public void setMotorTestSpeedNeg() {
        storageMotor.set(-testSpeed);
    }
    public void upSpeed() {
        testSpeed += 0.05;
    }
    public void downSpeed() {
        testSpeed -= 0.05;
    }
    public void testSpeedShutdown() {
        testSpeed = 0;
    }
    public void shutdown(){
        storageMotor.set(0);
    }
}