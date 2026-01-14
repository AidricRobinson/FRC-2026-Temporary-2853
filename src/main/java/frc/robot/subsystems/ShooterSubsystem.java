package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class ShooterSubsystem extends SubsystemBase {
  private SparkFlex leftMotor; 
  private SparkFlex rightMotor;
  private SparkFlexConfig leftMotorConfig;
  private SparkFlexConfig rightMotorConfig;
  double testSpeed = 0;

  
  public ShooterSubsystem() {
    leftMotor = new SparkFlex(PortConstants.leftMotorPort, MotorType.kBrushless);
   rightMotor = new SparkFlex(PortConstants.rightMotorPort, MotorType.kBrushless);

    leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   rightMotorConfig = new SparkFlexConfig();
   rightMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
     
   rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    leftMotor.set(power);
   rightMotor.set(power);
  }
  public void setLeftPower(double power){
    leftMotor.set(power);
  }
  public void setRightPower(double power){
    leftMotor.set(power);
  }


  public void shooterTestSpeedUp(){
    testSpeed += 0.1;
}
public void shooterTestSpeedDown(){
    testSpeed -= 0.1;
}
public void shooterTestSpeedShutdown(){
    testSpeed = 0;
}
public double getShooterTestSpeed(){
    return testSpeed;
}
public void setShooterTestPower(){
    setPower(testSpeed);
}
public void shutdown(){
    leftMotor.set(0);
   rightMotor.set(0);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterTestSpeed", getShooterTestSpeed());
    SmartDashboard.updateValues();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
