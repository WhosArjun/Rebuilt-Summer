public class Shooter extends SubsystemBase{
    public TalonFX shooterMotor;
    public TalonFX indexMotor;

    public Shooter(int shooterMotorID, int indexMotorID){
        this.shooterMotor = new TalonFX(shooterMotorID);
        this.indexMotor = new TalonFX(indexMotorID);
        this.shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        this.indexMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    
}
