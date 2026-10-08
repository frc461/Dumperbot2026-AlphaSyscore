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
import org.wpilib.smartdashboard.SmartDashboard;
//import com.revrobotics.AbsoluteEncoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.constants.*;

import org.wpilib.command2.SubsystemBase;

public class Launcher extends SubsystemBase {
    
    private final TalonFX FlywheelA = new TalonFX(41);
    private final TalonFX FlywheelB = new TalonFX(40);
    private final TalonFX FlywheelC = new TalonFX(43);
    private final TalonFX FlywheelD = new TalonFX(44);
    private final TalonFX Kicker1 = new TalonFX(50);
    private final TalonFX Kicker2 = new TalonFX(53);
    private final TalonFX HoodKraken = new TalonFX(51);
    
    private double targetRollerRPM = 0.0;
    private double targetHoodPosition = 0.0;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final PositionVoltage positionControl = new PositionVoltage(0);
 
    private final CANcoder hoodAbsoluteEncoder = new CANcoder(58);
    


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
        


        MainRoller1.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        MainRoller1.getConfigurator().apply(config);

        MainRoller2.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        MainRoller2.getConfigurator().apply(config);

        MainRoller3.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        MainRoller3.getConfigurator().apply(config);

        MainRoller4.getConfigurator().apply(new TalonFXConfiguration());
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.Slot0.kP = .4;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.2056;
        MainRoller4.getConfigurator().apply(config);

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

        SmartDashboard.putNumber(KEY_HOOD_ANGLE, 0.0);
        SmartDashboard.putNumber(KEY_FLY_RPM, 0.0);

        // SmartDashboard.putNumber("Tuning Kp", lastP);
        // SmartDashboard.putNumber("Tuning Ki", lastI);
        // SmartDashboard.putNumber("Tuning Kd", lastD);        
    }

    private final VoltageOut voltageControl = new VoltageOut(0);

    public void setVoltage(double volts) {
        MainRoller1.setControl(voltageControl.withOutput(volts));
    }

    public void setMainRoller2Voltage(double volts) {
        MainRoller2.setControl(voltageControl.withOutput(volts));
    }

    public void setMainRoller3Voltage(double volts) {
        MainRoller3.setControl(voltageControl.withOutput(volts));
    }

    public void setMainRoller4Voltage(double volts) {
        MainRoller4.setControl(voltageControl.withOutput(volts));
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
        MainRoller1.setControl(velocityControl.withVelocity(rps) );
        MainRoller2.setControl(velocityControl.withVelocity(rps) );
        MainRoller3.setControl(velocityControl.withVelocity(rps) ); 
        MainRoller4.setControl(velocityControl.withVelocity(rps) );   
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
        MainRoller1.stopMotor();
        MainRoller2.stopMotor();
        MainRoller3.stopMotor();
    }

    private static final String KEY_FLY_RPM = "Launcher RPM";
    private static final String KEY_HOOD_ANGLE = "Hood ANG" ;
    

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Flywheel Actual RPM", MainRoller1.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Flywheel Target RPM", targetRollerRPM);
        SmartDashboard.putNumber("Flywheel Temperature", MainRoller1.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood Position", HoodKraken.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder Position", hoodAbsoluteEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("convertHoodPosition", convertHoodPosition(1.25));

        
        double hoodAngle = SmartDashboard.getNumber(KEY_HOOD_ANGLE, 0.0);
        double launcherRPM = SmartDashboard.getNumber(KEY_FLY_RPM, 0.0);
       
        
        // setFlywheelVelocity(launcherRPM);
        // setHoodPosition(hoodAngle);
    }
}




