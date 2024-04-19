package org.team2168.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;

public class SetPipeline extends Command {
    
    private Limelight limelight;
    private int pipeline;
    private boolean isPipelineSet = false;
    private LEDs leds;
    double limeErrorTolerance = 1.0; //in degrees
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

        // if (Math.abs(limelight.getOffsetX()) < limeErrorTolerance) {
        //     leds.redlight(true);
        //   }
        //     else {
        //       leds.redlight(false);}
    } 
            

    public boolean isFinished() {
        return isPipelineSet;
    }
}
