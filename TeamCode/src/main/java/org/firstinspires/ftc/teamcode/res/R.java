// NOTE: This hacky mess exists because Sloth doesn't like R files
//       The value will need to be changed whenever arcs is changed.
//       For future reference, the value was taken from 
//
//     TeamCode\build\intermediates\runtime_symbol_list\debug\processDebugResources\R.txt
//
// TODO: Move this to another subproject (e.g FtcRobotController) so that its APK that 
//       is slow installed has the R file that contains arcs, and our TeamCode source
//       references that instead? Hacky, but more maintainable
package org.firstinspires.ftc.teamcode.res;

public final class R {
  public static final class raw {
    public static final int arcs=0x7f100000;
  }
}