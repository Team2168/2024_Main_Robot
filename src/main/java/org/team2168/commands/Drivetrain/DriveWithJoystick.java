package org.team2168.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;

public class DriveWithJoystick extends Command {

    private OI oi;
    private SlewRateLimiter rotationRateLimiter;
    private Drivetrain drivetrain;
    private double chassisRot = 0.0;
    private double kDriveInvert = 1.0;


    public DriveWithJoystick(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
    
        this.drivetrain = drivetrain;
    
        addRequirements(drivetrain);
      }

    // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        oi = OI.getInstance();
        rotationRateLimiter = new SlewRateLimiter(0.9);
      }

      public void execute() {
      // chooses button or joystick option for rotating chassis
        if (OI.joystickChooser.getSelected().equals("flight")) {
          if (oi.driverJoystick.isPressedButtonA()) {
            chassisRot = 0.4;
          }
          else if (oi.driverJoystick.isPressedButtonB()) {
            chassisRot = -0.4;
          }
          else {
            chassisRot = 0.0;
          }
        }
        else {
          chassisRot = oi.getDriverJoystickZValue();
        }

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          kDriveInvert = -1.0;
        }

        if (SmartDashboard.getString("Control Mode", "Joystick").equals("Joystick")) {
          drivetrain.drive(oi.getLimitedDriverJoystickYValue() * kDriveInvert, oi.getLimitedDriverJoystickXValue() * kDriveInvert, rotationRateLimiter.calculate(chassisRot));
        }
        else {
          drivetrain.stop();
          // drivetrain.drive(SmartDashboard.getNumber("Drive Forward", 0.0), SmartDashboard.getNumber("Drive Strafe", 0.0), SmartDashboard.getNumber("Drive Azimuth", 0.0));
        }
        SmartDashboard.putNumber("Joystick Y", oi.getDriverJoystickYValue());
        SmartDashboard.putNumber("Joystick X", oi.getDriverJoystickXValue());
        SmartDashboard.putNumber("Joystick Z", oi.getDriverJoystickZValue());
      }
    


    
}
