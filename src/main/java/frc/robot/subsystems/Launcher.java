package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut; 
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.telemetry.Telemetry;

//import com.revrobotics.AbsoluteEncoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.constants.*;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

public class Launcher extends SubsystemBase {
    
    private final TalonFX FlywheelA = new TalonFX(41, new CANBus(CANPort.CAN_S0));
    private final TalonFX FlywheelB = new TalonFX(40, new CANBus(CANPort.CAN_S0));
    private final TalonFX FlywheelC = new TalonFX(43, new CANBus(CANPort.CAN_S0));
    private final TalonFX FlywheelD = new TalonFX(44, new CANBus(CANPort.CAN_S0));
    private final TalonFX Kicker1 = new TalonFX(50, new CANBus(CANPort.CAN_S0));
    private final TalonFX Kicker2 = new TalonFX(53, new CANBus(CANPort.CAN_S0));
    private final TalonFX HoodKraken = new TalonFX(51, new CANBus(CANPort.CAN_S0));
    
    private double targetRollerRPM = 0.0;
    private double targetHoodPosition = 0.0;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final PositionVoltage positionControl = new PositionVoltage(0);
 
    private final CANcoder hoodAbsoluteEncoder = new CANcoder(58, new CANBus(CANPort.CAN_S0));
    


    public Launcher() {

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = Constants.LauncherConstants.ABSOLUTE_ENCODER_OFFSET;
        hoodAbsoluteEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration config = new TalonFXConfiguration();
        HoodKraken.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = 5;
        config.Slot0.kI = 0.2;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.8;
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // config.Feedback.FeedbackRemoteSensorID = hoodAbsoluteEncoder.getDeviceID();
        // config.Feedback.SensorToMechanismRatio = 1.0;
        // config.Feedback.RotorToSensorRatio = 4.0;
        HoodKraken.getConfigurator().apply(config);
        HoodKraken.setPosition(0);
        


        FlywheelA.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelA.getConfigurator().apply(config);

        FlywheelB.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelB.getConfigurator().apply(config);

        FlywheelC.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelC.getConfigurator().apply(config);

        FlywheelD.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        FlywheelD.getConfigurator().apply(config);

        Kicker1.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        Kicker1.setControl(
            new Follower(2, MotorAlignmentValue.Aligned)
        );

        Kicker2.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        Kicker2.setControl(
            new Follower(2, MotorAlignmentValue.Aligned)
        );

        Telemetry.log(KEY_HOOD_ANGLE, 0.0);
        Telemetry.log(KEY_FLY_RPM, 0.0);

        // SmartDashboard.putNumber("Tuning Kp", lastP);
        // SmartDashboard.putNumber("Tuning Ki", lastI);
        // SmartDashboard.putNumber("Tuning Kd", lastD);        
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        FlywheelA.setControl(voltageControl.withOutput(volts));
    }

    public void setFlywheelBVoltage(double volts) {
        FlywheelB.setControl(voltageControl.withOutput(volts));
    }

    public void setFlywheelCVoltage(double volts) {
        FlywheelC.setControl(voltageControl.withOutput(volts));
    }

    public void setFlywheelDVoltage(double volts) {
        FlywheelD.setControl(voltageControl.withOutput(volts));
    }

    public void setKicker1Voltage(double volts) {
        Kicker1.setControl(voltageControl.withOutput(volts));

    }

    public void setKicker2Voltage(double volts) {
        Kicker2.setControl(voltageControl.withOutput(volts));

    }

    public void setRollerVelocity(double RPM) {
        this.targetRollerRPM = RPM;
        
    }
    public void runRollers() {
        double rps = targetRollerRPM / 60.0;
        FlywheelA.setControl(velocityControl.withVelocity(rps) );
        FlywheelB.setControl(velocityControl.withVelocity(rps) );
        FlywheelC.setControl(velocityControl.withVelocity(rps) ); 
        FlywheelD.setControl(velocityControl.withVelocity(rps) );   
    }

    public void setHoodPosition(double pose) {
        this.targetHoodPosition = convertHoodPosition(pose);
        
    }

    public double convertHoodPosition(double pose) {
        double currentHoodEncoderPose = hoodAbsoluteEncoder.getPosition().getValueAsDouble();
        double encoderPose = pose / Constants.LauncherConstants.ENCODER_CONVERSION;
        double currentHoodMotorPose = HoodKraken.getPosition().getValueAsDouble();
        return ((currentHoodEncoderPose + encoderPose) * Constants.LauncherConstants.ENCODER_CONVERSION) + currentHoodMotorPose;
    }



    public void runHood() {
        HoodKraken.setControl(positionControl.withPosition(targetHoodPosition));
    }
    
    
    public void setKickerVelocity(double RPM) {
        Kicker1.setControl(velocityControl.withVelocity(RPM / 60.0));
    }

    public void setKicker2Velocity(double RPM) {
        Kicker2.setControl(velocityControl.withVelocity(RPM / 60.0));
    }

    public void shuttle() {
        setRollerVelocity(-2500);
        runRollers();
        setHoodPosition(2.65);
        runHood();
    }

    public void stopMainRollers() {
        FlywheelA.stopMotor();
        FlywheelB.stopMotor();
        FlywheelC.stopMotor();
        FlywheelD.stopMotor();
    }

    private static final String KEY_FLY_RPM = "Launcher RPM";
    private static final String KEY_HOOD_ANGLE = "Hood ANG" ;
    

    @Override
    public void periodic() {

        Telemetry.log("Flywheel Actual RPM", FlywheelA.getVelocity().getValueAsDouble() * 60.0);
        Telemetry.log("Flywheel Target RPM", targetRollerRPM);
        Telemetry.log("Flywheel Temperature", FlywheelA.getDeviceTemp().getValueAsDouble());
        Telemetry.log("Hood Position", HoodKraken.getPosition().getValueAsDouble());
        Telemetry.log("Encoder Position", hoodAbsoluteEncoder.getPosition().getValueAsDouble());
        Telemetry.log("convertHoodPosition", convertHoodPosition(1.25));

        
        // double hoodAngle = SmartDashboard.getNumber(KEY_HOOD_ANGLE, 0.0);
        // double launcherRPM = SmartDashboard.getNumber(KEY_FLY_RPM, 0.0);
       
        
        // setFlywheelVelocity(launcherRPM);
        // setHoodPosition(hoodAngle);
    }
}




