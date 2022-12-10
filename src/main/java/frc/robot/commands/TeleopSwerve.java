package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private int lBumperChannel;
    private int rBumperChannel;

    private static final Translation2d center = new Translation2d();
    private static final Translation2d rearLeftCorner = new Translation2d(-Constants.Swerve.trackWidth/2, -Constants.Swerve.wheelBase/2);
    private static final Translation2d rearRightCorner = new Translation2d(Constants.Swerve.trackWidth/2, -Constants.Swerve.wheelBase/2);

    // current center of rotation
    private Translation2d cor = center;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, int lBumperChannel, int rBumperChannel, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.lBumperChannel = lBumperChannel;
        this.rBumperChannel = rBumperChannel;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        boolean lBumper = controller.getRawButton(lBumperChannel);
        boolean rBumper = controller.getRawButton(rBumperChannel);

        if (lBumper){
            cor = rearLeftCorner;
            rAxis = -1;
        } else if (rBumper) {
            cor = rearRightCorner;
            rAxis = 1;
        } else {
            cor = center;
        }


        SmartDashboard.putNumber("raw xAxis", xAxis);
        SmartDashboard.putNumber("raw yAxis", yAxis);
        SmartDashboard.putNumber("raw rAxis", rAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        SmartDashboard.putNumber("new xAxis", xAxis);
        SmartDashboard.putNumber("new yAxis", yAxis);
        SmartDashboard.putNumber("new rAxis", rAxis);

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop, cor);
    }
}

