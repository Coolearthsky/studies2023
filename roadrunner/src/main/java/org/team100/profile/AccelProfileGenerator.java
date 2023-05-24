// package org.team100.profile;

// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
// import com.acmerobotics.roadrunner.profile.MotionState;

// /**
//  * arg, this is because Roadrunner makes the accel generator private, but i need it for the manual case.
//  * 
//  * So i translated the Kotlin.
//  */
// public class AccelProfileGenerator {
    
//     public MotionProfile 

//  generateAccelProfile(
// MotionState        start ,
//         double maxVel ,
//          double maxAccel,
//         double maxJerk
//     ) {

//             // jerk-limited
//             // compute the duration and velocity of the first segment
//             double deltaT1 = 0;
//             double deltaV1 = 0;
//              if (start.getA() > maxAccel) {
//                 // slow down and see where we are
//                  deltaT1 = (start.getA() - maxAccel) / maxJerk;
//                  deltaV1 = start.getA() * deltaT1 - 0.5 * maxJerk * deltaT1 * deltaT1;
//             } else {
//                 // otherwise accelerate
//                  deltaT1 = (maxAccel - start.getA()) / maxJerk;
//                  deltaV1 = start.getA() * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1;
//             }

//             // compute the duration and velocity of the third segment
//             double deltaT3 = maxAccel / maxJerk;
//             double deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3;

//             // compute the velocity change required in the second segment
//             double deltaV2 = maxVel - start.getV() - deltaV1 - deltaV3;

//             if (deltaV2 < 0.0) {
//                 // there is no constant acceleration phase
//                 // the second case checks if we're going to exceed max vel
//                 if (start.getA() > maxAccel || (start.getV() - maxVel) > (start.getA() * start.getA()) / (2 * maxJerk)) {
//                     // problem: we need to cut down on our acceleration but we can't cut our initial decel
//                     // solution: we'll lengthen our initial decel to -max accel and similarly with our final accel
//                     // if this results in an over correction, decel instead to a good accel
//                     double newDeltaT1 = (start.getA() + maxAccel) / maxJerk;
//                     double newDeltaV1 = start.getA() * newDeltaT1 - 0.5 * maxJerk * newDeltaT1 * newDeltaT1;

//                     double newDeltaV2 = maxVel - start.getV() - newDeltaV1 + deltaV3;

//                     if (newDeltaV2 > 0.0) {
//                         // we decelerated too much
//                         var roots = solveQuadratic(
//                             -maxJerk,
//                             2 * start.getA(),
//                             start.getV() - maxVel - start.getA() * start.getA() / (2 * maxJerk)
//                         )
//                         var finalDeltaT1 = roots.filter { it >= 0.0 }.minOrNull()!!
//                         var finalDeltaT3 = finalDeltaT1 - start.getA() / maxJerk

//                         MotionProfileBuilder(start)
//                                 .appendJerkControl(-maxJerk, finalDeltaT1)
//                                 .appendJerkControl(maxJerk, finalDeltaT3)
//                                 .build();
//                     } else {
//                         // we're almost good
//                         var newDeltaT2 = newDeltaV2 / -maxAccel;

//                         MotionProfileBuilder(start)
//                                 .appendJerkControl(-maxJerk, newDeltaT1)
//                                 .appendJerkControl(0.0, newDeltaT2)
//                                 .appendJerkControl(maxJerk, deltaT3)
//                                 .build();
//                     }
//                 } else {
//                     // cut out the constant accel phase and find a shorter delta t1 and delta t3
//                     var roots = solveQuadratic(
//                         maxJerk,
//                         2 * start.getA(),
//                         start.getV() - maxVel + start.getA() * start.getA() / (2 * maxJerk)
//                     );
//                     var newDeltaT1 = roots.filter { it >= 0.0 }.minOrNull()!!
//                     var newDeltaT3 = newDeltaT1 + start.getA() / maxJerk;

//                     MotionProfileBuilder(start)
//                             .appendJerkControl(maxJerk, newDeltaT1)
//                             .appendJerkControl(-maxJerk, newDeltaT3)
//                             .build();
//                 }
//             } else {
//                 // there is a constant acceleration phase
//                 var deltaT2 = deltaV2 / maxAccel;

//                 var builder = new MotionProfileBuilder(start);
//                 if (start.getA() > maxAccel) {
//                     builder.appendJerkControl(-maxJerk, deltaT1);
//                 } else {
//                     builder.appendJerkControl(maxJerk, deltaT1);
//                 }
//                 builder.appendJerkControl(0.0, deltaT2)
//                         .appendJerkControl(-maxJerk, deltaT3)
//                         .build();
//             }
        
//     }
// }
