//                                                 @
//                                                  &@@
//                          * .                    * @@@
//                           * (@   ,                 @@@@
//                               @@@*       /          @@@@
//                                @@@@@@    @@(     ,* ,@@@@@
//                         %@@@@/*  @@@@@@@@       ,**. @@@@@@
//                      #********,    @@@@@@@@@@    ***  @@@@@@
//                   **********    /    @@@@@@@@@@@@   ,  @@@@@@
//                              &@@/  (@  (@@@@@@@@@@@@   @@@@@@@
//                            @@@@@//  @@@@@@@@@@@@@@@@@@& @@@@@@@
//                          @@@@@@@//  @@@@@@@@# .@@@@@@@@@@@@@@@@
//                         @@@@@@&///  %@@@@@@@@(  *  @@@@@@@@@@@@
//                       *@@@@@//   @@@@@@@@@@@@@@%     @@@@@@@@@@@
//                      .@@@@@@@@@@//   .@@@@@@@@@@@@@@  @@@@@@@@@@@
//                      @@@@@@@@@@@@@@(/     @@@@@@@@@@@@@@@@@@@@@@@@@
//                   @ %@@@@@@@@@@@@@@   ,  @@@@@@@@@@@@@@@@@@@@@@@@@@@
//                  @@ @@@@@@@@@@@@@   .             *@@@@@@@@@  @@@@@@#
//                 @@@ @@@@@@@@@@@@%   *******@@@&///     &@@@@@@@@@@@@@
//                 @**  @@@@@@@@@@@   ******@@@@@@,          @@@@@@@@@@
//                 #*** @@@@@@@@@@@   *****@@@@@                  @@@@*
//                ***   @@@@@@@@@@@  ,****@@@,
//                 *      @@@@@@@@@@.  *****@@
//                          @@@@@@@@@#   ***%@
//                           ,@@@@@@@@@    ***@,  /
//                              @@@@@@@@@(    ***   //////*.     */
//                                 //@@@@@@%/      *    ///////
//                                 @    //////////
//                                   @@**
//                                       @*****
//                                             *

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.TensorVision;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Box {
    @JsonProperty("xmin")
    public int xMin;

    @JsonProperty("ymin")
    public int yMin;

    @JsonProperty("xmax")
    public int xMax;

    @JsonProperty("ymax")
    public int yMax;
}
