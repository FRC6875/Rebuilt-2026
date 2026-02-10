package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenSubsystem extends SubsystemBase {

    private final TalonFX krakenMotor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

    public KrakenSubsystem(int canID) {
        krakenMotor = new TalonFX(canID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Closed-loop PID values (TUNE THESE)
        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kV = 0.0;

        // Optional: current limiting
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        krakenMotor.getConfigurator().apply(config);
    }

    /**
     * Set desired position in motor rotations
     */
    public void setPosition(double rotations) {
        krakenMotor.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return krakenMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        krakenMotor.stopMotor();
    }
}