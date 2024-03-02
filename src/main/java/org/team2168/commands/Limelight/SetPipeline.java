package org.team2168.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.subsystems.Limelight;

public class SetPipeline extends CommandBase {
    
    private Limelight limelight;
    private int pipeline;
    private boolean isPipelineSet = false;

    public SetPipeline(Limelight limelight, int pipeline) {

        this.limelight = limelight;
        this.pipeline = pipeline;


    }

    @Override
    public void initialize() {
        limelight.enableBaseCameraSettings();
    }

    @Override
    public void execute() {
        limelight.setPipeline(pipeline);
        isPipelineSet = true;
    }

    public boolean isFinished() {
        return isPipelineSet;
    }
}
