package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class SetKrakenPosition extends Command {

    private final KrakenSubsystem kraken;
    private final double targetRotations;

    public SetKrakenPosition(KrakenSubsystem kraken, double targetRotations) {
        this.kraken = kraken;
        this.targetRotations = targetRotations;
        addRequirements(kraken);
    }

    @Override
    public void initialize() {
        kraken.setPosition(targetRotations);
    }

    @Override
    public boolean isFinished() {
        // End when we're close enough
        return Math.abs(kraken.getPosition() - targetRotations) < 0.02;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            kraken.stop();
        }
    }
}