package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class Intake extends SubsystemBase {

    private static final double RETRACTED_POSITION = 0.1;
    private static final double  EXTENDED_POSITION = -14;
    // Stop commanding position control once the deploy is this close to its target (rotor rotations).
    private static final double DEPLOY_POSITION_TOLERANCE = 0.2;
    private static final double MANUAL_DEPLOY_VOLTS = 2.0;
    private static final double SLOW_FEED_VOLTS = 3.0;
    private static final double FEED_IN_SECONDS = 0.5;
    private static final double FEED_OUT_SECONDS = 0.5;
    private static final double HOMING_VOLTAGE = 2.0;
    private static final double HOMING_STATOR_CURRENT = 45.0;
    private static final int HOMING_STALL_CYCLES = 8;
    private static final int HOMING_TIMEOUT_CYCLES = 150;
    
    private final TalonFX DeployKraken = new TalonFX(42, new CANBus(CANPort.CAN_S4));
    private final TalonFX IntakeRollerKrakenA = new TalonFX(59, new CANBus(CANPort.CAN_S4));
    private final TalonFX IntakeRollerKrakenB = new TalonFX(56, new CANBus(CANPort.CAN_S4));
    
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private boolean homing = false;
    private int homingCycles = 0;
    private int stalledCycles = 0;
    private boolean deployPositionActive = false;
    private double deployTargetPosition = RETRACTED_POSITION;


    public Intake() {
        configureMotor(DeployKraken);
        configureMotor(IntakeRollerKrakenA);
        configureMotor(IntakeRollerKrakenB);
        DeployKraken.setPosition(0.0);
        
    }



    private void configureMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration(); 
        config.CurrentLimits.SupplyCurrentLimit = 40.0; 
        config.CurrentLimits.SupplyCurrentLowerLimit = 30.0; 
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1; 
        config.CurrentLimits.StatorCurrentLimit = 60.0; 
        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        motor.getConfigurator().apply(config); 
    }


    public void setIntakeVoltage() {
        IntakeRollerKrakenA.setControl(voltageControl.withOutput(16.0));
        IntakeRollerKrakenB.setControl(voltageControl.withOutput(-16.0));
    }

    public void runIntake() {
        setIntakeVoltage();
    }

    public void runIntakeSlow() {
        IntakeRollerKrakenA.setControl(voltageControl.withOutput(SLOW_FEED_VOLTS));
        IntakeRollerKrakenB.setControl(voltageControl.withOutput(-SLOW_FEED_VOLTS));
    }

    public void runIntakeSlowOut() {
        IntakeRollerKrakenA.setControl(voltageControl.withOutput(-SLOW_FEED_VOLTS));
        IntakeRollerKrakenB.setControl(voltageControl.withOutput(SLOW_FEED_VOLTS));
    }

    /** Runs the intake rollers slowly inward, then briefly reverses them. */
    public Command slowInThenOut() {
        return Commands.sequence(
            Commands.runEnd(this::runIntakeSlow, this::stopRollers, this).withTimeout(FEED_IN_SECONDS),
            Commands.runEnd(this::runIntakeSlowOut, this::stopRollers, this).withTimeout(FEED_OUT_SECONDS)
        );
    }

    public Command retractAndSlowInThenOut() {
        return Commands.sequence(
            runOnce(this::retractIntake),
            slowInThenOut()
        );
    }

    private void stopRollers() {
        IntakeRollerKrakenA.stopMotor();
        IntakeRollerKrakenB.stopMotor();
    }

    public void manualDeployIn() {
        setDeployVoltage(MANUAL_DEPLOY_VOLTS);
    }

    public void manualDeployOut() {
        setDeployVoltage(-MANUAL_DEPLOY_VOLTS);
    }

    public void stopIntake() {
        DeployKraken.stopMotor();
        IntakeRollerKrakenA.stopMotor();
        IntakeRollerKrakenB.stopMotor();
    }

    public void setDeployVoltage(double volts) {
        deployPositionActive = false;
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

    public void extendIntake() {
        moveDeployTo(EXTENDED_POSITION);
    }

    public void retractIntake() {
        moveDeployTo(RETRACTED_POSITION);
    }

    private void moveDeployTo(double targetPosition) {
        deployTargetPosition = targetPosition;
        deployPositionActive = true;
        updateDeployPosition();
    }

    private void updateDeployPosition() {
        if (!deployPositionActive) return;
        double error = deployTargetPosition - DeployKraken.getPosition().getValueAsDouble();
        if (Math.abs(error) <= DEPLOY_POSITION_TOLERANCE) {
            DeployKraken.stopMotor();
            deployPositionActive = false;
            return;
        }
        DeployKraken.setControl(positionControl.withPosition(deployTargetPosition));
    }

    public void stopDeployment() {
        deployPositionActive = false;
        DeployKraken.stopMotor();
    }

    

    public void ExtendIntake() {
        extendIntake();
    }

    public void RetractIntake() {
        retractIntake();
    }
     @Override
    public void periodic() {
        updateDeployPosition();
        // if (homing) {
        //     homingCycles++;
        //     DeployKraken.setControl(voltageControl.withOutput(-HOMING_VOLTAGE));

        //     if (DeployKraken.getStatorCurrent().getValueAsDouble() >= HOMING_STATOR_CURRENT) {
        //         stalledCycles++;
        //     } else {
        //         stalledCycles = 0;
        //     }

        //     if (stalledCycles >= HOMING_STALL_CYCLES || homingCycles >= HOMING_TIMEOUT_CYCLES) {
        //         DeployKraken.stopMotor();
        //         if (stalledCycles >= HOMING_STALL_CYCLES) {
        //             DeployKraken.setPosition(RETRACTED_POSITION);
        //         }
        //         homing = false;
        //     }
        // }

        // Telemetry.log("Intake Position", DeployKraken.getPosition().getValueAsDouble());
        // Telemetry.log("Intake Homing", homing);
    }
}

