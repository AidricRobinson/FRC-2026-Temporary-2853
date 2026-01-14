package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.PortConstants;

public class StorageSubsystem extends SubsystemBase {
    private SparkFlex storageMotor;
    private double testSpeed;
    private SparkFlexConfig storageMotorConfig;
    public StorageSubsystem(){
        storageMotor = new SparkFlex(PortConstants.StorageMotorPort, MotorType.kBrushless);
        testSpeed= 0.2;
        storageMotorConfig = new SparkFlexConfig();
        storageMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        storageMotor.configure(storageMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public double getTestSpeed(){
        return testSpeed;
    }
    public void shutdown(){
        storageMotor.set(0);
    }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("StorageTestSpeed", getTestSpeed());
    SmartDashboard.putNumber("StorageSliderTestingSpeed", testSpeed);
    SmartDashboard.updateValues();
  }
}