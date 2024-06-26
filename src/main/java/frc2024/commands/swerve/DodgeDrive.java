package frc2024.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Constants.SwerveConstants;
import frc2024.Constants.SwerveConstants.ModuleConstants.ModuleLocation;
import frc2024.subsystems.swerve.Swerve;

public class DodgeDrive extends Command{

    private Swerve swerve;
    private DoubleSupplier[] translationSup;
    private DoubleSupplier rotationSup;
    private DodgeDirection dodgeDirection;
    private BooleanSupplier slowModeSup;
    private Rotation2d lastHeading;
    
    public DodgeDrive(Swerve swerve, DoubleSupplier[] translationSup, DoubleSupplier rotationSup, DodgeDirection direction, BooleanSupplier slowMode) {
        addRequirements(swerve);
        setName("DodgeDrive");

        this.swerve = swerve;
        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.dodgeDirection = direction;
        this.slowModeSup = slowMode;
    }

    @Override
    public void initialize() {
        lastHeading = swerve.getEstimatedHeading();
    }

    @Override
    public void execute() {
        Translation2d translationValue = 
            slowModeSup.getAsBoolean() 
            ? new Translation2d(translationSup[0].getAsDouble()*0.5, translationSup[1].getAsDouble()*0.5).times(SwerveConstants.MAX_SPEED * AllianceFlipUtil.getDirectionCoefficient())
            : new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED * AllianceFlipUtil.getDirectionCoefficient());
        double rotationValue = rotationSup.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;
        
        ChassisSpeeds targetSpeeds;
        targetSpeeds = swerve.fieldRelativeSpeeds(translationValue, rotationValue);
        
        swerve.setChassisSpeeds(targetSpeeds, getDodgeLocation(), true);
    }

    public Translation2d getDodgeLocation(){
        Rotation2d heading = lastHeading;
        if(ScreamUtil.valueBetween(heading.getDegrees(), 45, -45)){
            if(dodgeDirection == DodgeDirection.LEFT) return ModuleLocation.FRONT_LEFT.translation;
            return ModuleLocation.FRONT_RIGHT.translation;
        } else if(ScreamUtil.valueBetween(heading.getDegrees(), -45, -135)){
            if(dodgeDirection == DodgeDirection.LEFT) return ModuleLocation.BACK_LEFT.translation;
            return ModuleLocation.FRONT_LEFT.translation;
        } else if(ScreamUtil.valueBetween(heading.getDegrees(), -135, -179.99) || ScreamUtil.valueBetween(heading.getDegrees(), 179.99, 135)){
            if(dodgeDirection == DodgeDirection.LEFT) return ModuleLocation.BACK_RIGHT.translation;
            return ModuleLocation.BACK_LEFT.translation;
        } else if(ScreamUtil.valueBetween(heading.getDegrees(), 135, 45)){
            if(dodgeDirection == DodgeDirection.LEFT) return ModuleLocation.FRONT_RIGHT.translation;
            return ModuleLocation.BACK_RIGHT.translation;
        } else {
            return new Translation2d();
        }
    }

    public enum DodgeDirection{
        LEFT, RIGHT;
    }
}

