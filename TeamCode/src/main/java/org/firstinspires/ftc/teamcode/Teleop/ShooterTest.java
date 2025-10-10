package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter using DcMotorEx.setVelocity() with additional acceleration feedforward (kA).
 * Call update() frequently (e.g. each loop) after setTargetRPM().
 */
public class ShooterTest {

    private final DcMotorEx S1;
    private final DcMotorEx S2;

    // Encoder/motor constants
    private static final double TICKS_PER_REV = 112.0; // change if different
    private static final double MAX_RPM = 6000.0;

    // Feedforward / Acceleration feedforward
    // kS - static (overcome friction); kV - velocity ff; kA - acceleration ff
    // Units: kS and kV/kA are in motor power units (0..1 scale)
    private double kS = 0.02;    // small static offset
    private double kV = 0.0002;  // velocity feedforward (power per RPM)
    private double kA = 0.00003; // accel feedforward (power per RPM/sec) — tune to taste

    // Built-in PIDF (these run on the control hub)
    // Tune these (start with moderate P, low D)
    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(12.0, 0.0005, 4.0, 7.0);

    // State
    private double targetRPM = 0.0;
    private double lastTargetRPM = 0.0; // for computing desired accel
    private double lastUpdateTime = 0.0;

    private final ElapsedTime clock = new ElapsedTime();

    public ShooterTest(HardwareMap hardwareMap, String s1Name, String s2Name) {
        S1 = hardwareMap.get(DcMotorEx.class, s1Name);
        S2 = hardwareMap.get(DcMotorEx.class, s2Name);

        S1.setDirection(DcMotorSimple.Direction.REVERSE);
        S2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use RUN_USING_ENCODER so velocity PID runs
        S1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        S2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set internal PIDF
        S1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        S2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // init time
        lastUpdateTime = clock.seconds();
    }

    /**
     * Set the requested target RPM. Call this when you want to change speed.
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, MAX_RPM);
    }

    /**
     * Call this in your loop (e.g., while (opModeIsActive())) as often as possible.
     * It computes acceleration feedforward and applies setVelocity with ff.
     */
    public void update() {
        double now = clock.seconds();
        double dt = now - lastUpdateTime;
        if (dt <= 0) dt = 1e-6;

        // Compute desired acceleration (RPM per second) based on change in target
        double desiredAccelRPMperSec = (targetRPM - lastTargetRPM) / dt;

        // Optionally clamp accel to a sensible maximum to avoid huge feedforward spikes
        double maxAccel = 8000.0; // tune: maximum RPM/sec you'd ever want to request
        desiredAccelRPMperSec = Range.clip(desiredAccelRPMperSec, -maxAccel, maxAccel);

        // Compute feedforward power: kS + kV * targetRPM + kA * accel
        double ffPower = kS * Math.signum(targetRPM) + (kV * targetRPM) + (kA * desiredAccelRPMperSec);

        // Convert RPM -> ticks/sec for setVelocity
        double ticksPerSec = rpmToTicksPerSecond(targetRPM);

        // Apply velocity with feedforward
        S1.setVelocity(ticksPerSec);
        S2.setVelocity(ticksPerSec);

        // update bookkeeping
        lastTargetRPM = targetRPM;
        lastUpdateTime = now;
    }

    public void stopShooter() {
        targetRPM = 0;
        lastTargetRPM = 0;
        S1.setVelocity(0);
        S2.setVelocity(0);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        double avgTicksPerSec = (S1.getVelocity() + S2.getVelocity()) / 2.0;
        return ticksPerSecondToRPM(avgTicksPerSec);
    }

    // ------- helpers -------
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double ticksPerSecondToRPM(double ticksPerSec) {
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }

    // Expose tuners so you can change the FF constants at runtime if desired
    public void setFeedforwardGains(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public void setPIDFCoefficients(PIDFCoefficients coeffs) {
        this.pidfCoefficients = coeffs;
        S1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        S2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
}
