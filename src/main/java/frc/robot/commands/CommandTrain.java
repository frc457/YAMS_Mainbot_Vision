package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HopperSubsytem;


public class CommandTrain {
    
    // private ArmSubsystem m_arm;
    private IndexerSubsystem m_indexer;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
    private HopperSubsytem m_Hopper;

    public CommandTrain(//ArmSubsystem Arm, 
    IndexerSubsystem Indexer, 
                      IntakeSubsystem Intake,
                       ShooterSubsystem Shooter, HopperSubsytem Hopper){
       // m_arm = Arm;
        m_indexer = Indexer;
       m_intake = Intake;
        m_shooter = Shooter;
        m_Hopper = Hopper;

    }
    // +shooter = out
    //-shooter = in
    //+hopper/indexer = in
     //-hopper/indexer = out

    public Command shoot(){ 
        return m_shooter.setVelocity(RPM.of(5000)).withTimeout(1)
        .alongWith(m_indexer.set(-1).withTimeout(1)) 
        .andThen(
            m_intake.set(-1) 
             .alongWith(m_shooter.setVelocity(RPM.of(5000)))
            .alongWith(m_indexer.set(-1)) 
            .alongWith(m_Hopper.set(-1)));
        
       // .beforeStarting(() -> SmartDashboard.putBoolean("Shoot Running", true))
       // .finallyDo(interrupted -> SmartDashboard.putBoolean("Shoot Running", false));
    }


    public Command Intaking(){
        return m_intake.set(-1)
        .alongWith(m_indexer.set(-0.3))
        .beforeStarting(() -> SmartDashboard.putBoolean("Intaking", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Intaking", false));
    }

    public Command mixer(){
        return m_Hopper.set(1).withTimeout(0.5)
        .andThen(m_Hopper.set(-1).withTimeout(0.5))
        .andThen(m_Hopper.set(1).withTimeout(0.5))
        .beforeStarting(() -> SmartDashboard.putBoolean("Mixing", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Mixing", false));
    }
    

    public Command throwup(){
        return m_intake.set(1) 
            .alongWith(
                m_indexer.set(1)
                )
            .alongWith(m_shooter.setVelocity(RPM.of(500))) 
            .alongWith(m_Hopper.set(1))
            .beforeStarting(() -> SmartDashboard.putBoolean("Throwup", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Throwup", false));
    
    }


//     //Pathplanner Commands

//     public Command timedShoot(){
//         return m_shooter.setVelocity(RPM.of(500)).withTimeout(1)
//         .alongWith(m_Hopper.set(1)).withTimeout(1) 
//         .andThen(
//             m_intake.setVelocity(RPM.of(50)) 
//             .alongWith(m_indexer.set(1)) 
//             .alongWith(m_shooter.setVelocity(RPM.of(500))) 
//             .alongWith(m_Hopper.set(1)) 
//             .withTimeout(4)
//         );
//     }


//     public Command timedIntaking(){
//         return m_intake.setVelocity(RPM.of(100))
//         .alongWith(m_indexer.set(1))
//         .withTimeout(2);
//     }

//     public Command armOscillate() {
//         return m_arm.setAngle(Degrees.of(0)).withTimeout(0.5)
//             .andThen(m_arm.setAngle(Degrees.of(50))).withTimeout(0.5)
//             .repeatedly();

//     }
// }
}