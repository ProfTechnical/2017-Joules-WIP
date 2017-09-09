package org.usfirst.frc.team4910.iterations;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team4910.robot.*;
import org.usfirst.frc.team4910.util.CrashTracker;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Jason Cobb
 * This class handles everything that needs to iterate
 * This was stolen from team254, but I changed the name of the class so it's okay
 */
public class Iterator {
    public final double kPeriod = 0.01;

    private boolean isRunning;

    private final Notifier notifier_;
    private final List<Iterate> iterable;
    private final Object syncLock = new Object();
    private double timestamp = 0;
    private double dt = 0;
    private abstract class RunnableLogCrash implements Runnable{
        @Override
        public final void run() {
            try {
                runCrashTracked();
            } catch (Throwable t) {
                CrashTracker.logThrowableCrash(t);
                throw t;
            }
        }

        public abstract void runCrashTracked();
    }
    private final RunnableLogCrash runnable = new RunnableLogCrash() {
        @Override
        public void runCrashTracked() {
            synchronized (syncLock) {
                if (isRunning) {
                    double now = Timer.getFPGATimestamp();
                    for (Iterate iter : iterable) {
                        iter.run();
                    }
                    dt = now - timestamp;
                    timestamp = now;
                }
            }
        }
    };

    public Iterator() {
        notifier_ = new Notifier(runnable);
        isRunning = false;
        iterable = new ArrayList<>();
    }

    public synchronized void register(Iterate iter) {
        synchronized (syncLock) {
            iterable.add(iter);
        }
    }

    public synchronized void start() {
        if (!isRunning) {
            System.out.println("Starting loops");
            synchronized (syncLock) {
                timestamp = Timer.getFPGATimestamp();
                for (Iterate iter : iterable) {
                    iter.init();
                }
                isRunning = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (isRunning) {
            System.out.println("Stopping loops");
            notifier_.stop();
            synchronized (syncLock) {
                isRunning = false;
                for (Iterate iter : iterable) {
                    System.out.println("Stopping " + iter);
                    iter.end();
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt);
    }
	
}
