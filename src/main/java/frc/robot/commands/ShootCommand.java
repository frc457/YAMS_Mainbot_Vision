package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsytem;

public class ShootCommand extends Command
{

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private TransportSubsytem transport;
    private Supplier<AngularVelocity> setpoint;
    

    public ShootCommand(Supplier<AngularVelocity> shootSpeed, ShooterSubsystem shooter, IndexerSubsystem indexer, TransportSubsytem transport)
    {
        this.shooter = shooter;
        this.indexer = indexer;
        this.transport = transport;
        setpoint = shootSpeed;
    }
    
    
  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    shooter.setVelocity(setpoint.get());
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    shooter.setVelocity(setpoint.get());
    if(shooter.getVelocity().in(RPM) == setpoint.get().in(RPM))
    {
        indexer.set(1);
        transport.set(1);
    }else{
        indexer.set(0);
        transport.set(0);
    }
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    shooter.set(0);
  }
}
