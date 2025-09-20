package clarson.ftc.faker.updater;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;

/**
 * Indicates that a method will cause the automatic update of all updateables
 * registered in the same Updaters. Every updater the Updateable is registered
 * in will have an automatic update issued.
 */
@Documented
@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.METHOD)
@Inherited
public @interface SimulateDelay {
    public UpdatesWhen value();
}