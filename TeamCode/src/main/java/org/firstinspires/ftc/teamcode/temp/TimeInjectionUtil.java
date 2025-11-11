package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.time.Duration;
import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;


// TODO: Remove this file when we inevitably merge the HardwareFaker changes
/**
 * Contains methods meant to dependency inject certain time-keeping 
 * functionality in user code. `ElapsedTime`, `sleep()`, and `getRuntime()`
 * are all examples of features that are able to be injected by this class.
 *
 * The purpose of this is to remove reliance on the system clock time in user 
 * code. If any part of the code relies on the system clock, then testing will
 * take real time (often seconds). By removing reliance on such, test code is 
 * able to inject versions that rely on a simulated clock, which is able to be 
 * moved forward, moved backward, or paused as necessary.
 */
public class TimeInjectionUtil {
    private static Params defaultParams = new Params();
    private static Supplier<ElapsedTime> elapsedTimeHook = null;

    /**
     * Sets the parameters used when constructing a TimeInjectionUtil. This applies
     * to both the zero-argument constructor and the one-argument (OpMode) constructor
     * 
     * @param params The parameters to use when constructing new `TimeInjectionUtil`s.
     */
    public static void setDefaultParams(Params params) {
        defaultParams = params;
    }

    public static void setElapsedTimeGetter(Supplier<ElapsedTime> elapsedTimeSupplier) {
        elapsedTimeHook = elapsedTimeSupplier;
    }

    /**
     * Creates a new ElapsedTime, which may be an instanceof ElapsedTimeFake. 
     * If `registerElapsedTimesWithUpdaters()` has been called, this will return
     * ElapsedTimeFakes; otherwise, it will return ordinary ElapsedTimes.
     * 
     * This should be used in place of directly constructing an ElapsedTime in user 
     * code because this allows dependecy injection for ElapsedTimes, removing
     * reliance on the real clock and real timing, massively improving offline 
     * opmode test performance.
     * 
     * @return An ElapsedTime. An instance of ElapsedTimeFake after 
     * `registerElapsedTimesWithUpdaters()`
     */
    public static ElapsedTime getElapsedTime() {
        // Returning an ElapsedTimeFake if we have a non-null updater array
        if(elapsedTimeHook != null) {
            return elapsedTimeHook.get();
        }

        // Creating normal ElapsedTimes
        return new ElapsedTime();
    }

    /**
     * Parameters with which to use when constructing a TimeInjectionUtil.
     */
    public static class Params {
        private Supplier<OpMode> opModeTarget = () -> null;
        public Consumer<Long> opModeSleep = (Long millis) -> ((LinearOpMode) opModeTarget.get()).sleep(millis);
        public ThreadSleepMethod threadSleep = Thread::sleep;
        public Supplier<Double> getRuntime = () -> opModeTarget.get().getRuntime();
        public Runnable resetRuntime = () -> opModeTarget.get().resetRuntime();

        /**
         * Sets the opMode that will be used to supply the OpMode for 
         * `sleep()` & `getRuntime()`. Defaults to null.
         * 
         * NOTE: `threadSleep()` is unaffected by this method.
         */
        public Params setOpModeTarget(OpMode newOpModeTarget) {
            this.opModeTarget = () -> newOpModeTarget;
            return this;
        }

        /**
         * Injects the implementation for the `TimeInjectionUtil.sleep()` method.
         * This is not to be confused with `setThreadSleep()`. Where that injects
         * in place of `Thread.sleep()`, this injects for `OpMode.sleep()`.
         * Defaults to `opModeTarget.sleep()`.
         */
        public Params setSleep(Consumer<Long> sleepImplementation) {
            this.opModeSleep = sleepImplementation;
            return this;
        }
        
        /**
         * Injects the implementation for the `TimeInjectionUtil.threadSleep()` method.
         * This is not to be confused with `setSleep()`. Where that injects
         * in place of `OpMode.sleep()`, this injects for `Thread.sleep()`.
         * Defaults to `Thread.sleep()`.
         */
        public Params setThreadSleep(ThreadSleepMethod sleepImplementation) {
            this.threadSleep = sleepImplementation;
            return this;
        }

        public Params setGetRuntime(Supplier<Double> getRuntimeImplementation) {
            this.getRuntime = getRuntimeImplementation;
            return this;
        }
        
        public Params setResetRuntime(Runnable resetRuntimeImplementation) {
            this.resetRuntime = resetRuntimeImplementation;
            return this;
        }
    }

    private final Supplier<OpMode> opModeTarget;
    private final Consumer<Long> opModeSleep;
    private final ThreadSleepMethod threadSleep;
    private final Supplier<Double> getRuntime;
    private final Runnable resetRuntime;

    /**
     * Constructs a new TimeInjectionUtil using the default Params parameters. 
     * Note that `sleep()`, `getRuntime()`, `resetRuntime()` will throw 
     * NullPointerExceptions because no OpMode target has been provided
     */
    public TimeInjectionUtil() {
        this(defaultParams);
    }

    /**
     * Constructs a new TimeInjectionUtil using the given OpMode as the 
     * Params's target. All other parameters of the Params are default. This does
     * not throw NullPointerExceptions like the zero argument form.
     */
    public TimeInjectionUtil(OpMode opModeTarget) {
        // We clone the default params so that we don't mutate the params
        this(new Params()
            .setOpModeTarget(opModeTarget)
            .setSleep(defaultParams.opModeSleep)
            .setThreadSleep(defaultParams.threadSleep)
            .setGetRuntime(defaultParams.getRuntime)
            .setResetRuntime(defaultParams.resetRuntime)
        );
    }

    protected TimeInjectionUtil(Params params) {
        this.opModeTarget = params.opModeTarget;
        this.opModeSleep = params.opModeSleep;
        this.threadSleep = params.threadSleep;
        this.getRuntime = params.getRuntime;
        this.resetRuntime = params.resetRuntime;
    }

    /**
     * Peforms "LinearOpMode.sleep()" unless the functionality has been injected. If
     * the default params has been changed (see `setDefaultParams()`) such that its
     * `setSleep()` method has been called, then this requires 
     * 
     * @param milliseconds How long to sleep, in milliseconds.
     */
    public void sleep(long milliseconds) {
        opModeSleep.accept(milliseconds);
    }

    public void threadSleep(Duration duration) throws InterruptedException {
        threadSleep(duration.toMillis(), duration.toNanosPart());
    }

    public void threadSleep(long milliseconds) throws InterruptedException {
        threadSleep(milliseconds, 0);
    }

    public void threadSleep(long milliseconds, int nanos) throws InterruptedException {
        threadSleep.accept(milliseconds, nanos);
    }

    public double getRuntime() {
        return getRuntime.get();
    }

    public void resetRuntime() {
        resetRuntime.run();
    }
}