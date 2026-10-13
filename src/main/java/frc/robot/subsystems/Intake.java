package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut; 
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.signals.NeutralModeValue;


import java.io.ObjectInputFilter.Config;
import frc.robot.constants.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;
//import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.SubsystemBase;

import org.wpilib.hardware.discrete.DigitalInput;
//import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonFX DeployKraken = new TalonFX(42, new CANBus(CANPort.CAN_S4));
    private final TalonFX IntakeRollerKrakenA = new TalonFX(59, new CANBus(CANPort.CAN_S4));
    private final TalonFX IntakeRollerKrakenB = new TalonFX(56, new CANBus(CANPort.CAN_S4));
    
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    public Intake() {
        configureMotor(DeployKraken);
        configureMotor(IntakeRollerKrakenA);
        configureMotor(IntakeRollerKrakenB);
        
    }



    private void configureMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration(); 
        DeployKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimit = 40.0; 
        config.CurrentLimits.SupplyCurrentLowerLimit = 30.0; 
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1; 
        config.CurrentLimits.StatorCurrentLimit = 60.0; 
        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        motor.getConfigurator().apply(config); 
        DeployKraken.getConfigurator().apply(config);
        DeployKraken.setPosition(0);
    }
    private final DigitalInput RightForwardLimitSwitch = new DigitalInput(0);
    private final DigitalInput LeftForwardLimitSwitch = new DigitalInput(1);
    private final DigitalInput RearLimitSwitch = new DigitalInput(2);

    public void setIntakeVoltage(double volts) {
        IntakeRollerKrakenA.setControl(voltageControl.withOutput(volts));
    }

    public void setDeployVoltage(double volts) {
        DeployKraken.setControl(voltageControl.withOutput(volts));
    }

        private final VelocityVoltage velocityControl = new VelocityVoltage(0);


    public void setVelocity(double rotationsPerSecond) {
        DeployKraken.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
        IntakeRollerKrakenA.setControl(
            velocityControl.withVelocity(rotationsPerSecond)
        );
    }

    public void CheckRearLimitSwitch() {
        if (!RearLimitSwitch.get()) {
            DeployKraken.setPosition(0);
        }
    }

    public void ResetExtension() {
        if (!RightForwardLimitSwitch.get() && !LeftForwardLimitSwitch.get()) {
            DeployKraken.setPosition(-70);
        }
    }

    public void ExtendIntake() {
        DeployKraken.setControl(positionControl.withPosition(-70)
        .withLimitForwardMotion(!RightForwardLimitSwitch.get() && !LeftForwardLimitSwitch.get())
        );
    }

    public void RetractIntake() {
        DeployKraken.setControl(positionControl.withPosition(0));
    }
     @Override
    public void periodic() {

        Telemetry.log("Right Forward Limit Switch", !RightForwardLimitSwitch.get());
        Telemetry.log("Left Forward Limit Switch", !LeftForwardLimitSwitch.get());
        Telemetry.log("Rear Limit Switch", !RearLimitSwitch.get());
        Telemetry.log("Intake Position", DeployKraken.getPosition().getValueAsDouble());
        CheckRearLimitSwitch();
        ResetExtension();

        
        
    }
}

