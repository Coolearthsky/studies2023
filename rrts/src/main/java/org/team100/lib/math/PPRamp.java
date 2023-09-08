// package org.team100.lib.math;

// /**
//  * This is a separate class because the C++ code did it that way.
//  * 
//  * the idea here is "parabolic ramp" that goes "parabola to parabola" ("PP").
//  * There's also parabola-linear-parabola for velocity-limited cases, TODO: look
//  * at that.
//  * 
//  * see KrisLibrary/planning/ParabolicRamp.cpp
//  */
// public class PPRamp {
//     public static class Real {
//         double v;

//         public Real(double v) {
//             this.v = v;
//         }
//     }

//     // this is from the PPRamp inner class.
//       //input
//   Real x0,dx0;
//   Real x1,dx1;

//   //calculated
//   Real a;
//   Real tswitch,ttotal;

//   // this is from the Parabolic1D class
//     /// Input
//     Real x0, dx0;
//     Real x1, dx1;

//     /// Calculated upon SolveX
//     Real tswitch1, tswitch2; // time to switch between ramp/flat/ramp
//     Real ttotal;
//     Real a1, v, a2; // accel of first ramp, velocity of linear section, accel of second ramp

//     // these are from ParabolicRampConfig

//       ///tolerance for time equality
//   const static Real EpsilonT = 1e-10;

//   ///tolerance for position equality
//   const static Real EpsilonX = 1e-10;

//   ///tolerance for velocity equality
//   const static Real EpsilonV = 1e-10;

//   ///tolerance for acceleration equality
//   const static Real EpsilonA = 1e-10;

// ///print an error
// #define PARABOLIC_RAMP_PERROR(...) fprintf(stderr,__VA_ARGS__)


  
// bool PPRamp::SolveMinAccel(Real endTime)
// {
//   Real switch1,switch2;
//   Real apn = CalcMinAccel(endTime,1.0,switch1);
//   Real anp = CalcMinAccel(endTime,-1.0,switch2);
//   //LOG4CXX_INFO(KrisLibrary::logger(),"Accel for parabola +-: "<<apn<<", parabola -+: "<<anp);
//   if(apn >= 0) {
//     if(anp >= 0 && anp < apn)  a = -anp;
//     else a = apn;
//   }
//   else if(anp >= 0) a = -anp;
//   else {
//     a=0;
//     tswitch = -1;
//     ttotal = -1;
//     return false;
//   }
//   ttotal = endTime;
//   if(a == apn) 
//     tswitch = switch1;
//   else
//     tswitch = switch2;

//   //debug
//   if(gValidityCheckLevel >= 1) {
//     Real t2mT = tswitch-ttotal;
//     if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),CheckEpsilonX)) {
//       if(gVerbose >= 1)
// 	PARABOLIC_RAMP_PERROR("PPRamp::SolveMinAccel: Numerical error, x mismatch!\n");
//       if(gVerbose >= 2) {
// 	PARABOLIC_RAMP_PERROR("Forward ramp: %g, backward %g, diff %g\n",x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch)-(x1+t2mT*dx1-0.5*a*Sqr(t2mT)));
// 	PARABOLIC_RAMP_PERROR("A+ = %g, A- = %g\n",apn,anp);
// 	PARABOLIC_RAMP_PERROR("ramp %g,%g -> %g, %g\n",x0,dx0,x1,dx1);
// 	PARABOLIC_RAMP_PERROR("Switch 1 %g, switch 2 %g, total %g\n",switch1,switch2,ttotal);
	
// 	{
// 	  Real sign = 1.0;
// 	  Real a=Sqr(endTime);
// 	  Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
// 	  Real c=-Sqr(dx1-dx0);
// 	  PARABOLIC_RAMP_PERROR("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
// 	  Real t1,t2;
// 	  int res = quadratic(a,b,c,t1,t2);
// 	  PARABOLIC_RAMP_PERROR("Solutions: %d, %g and %g\n",res,t1,t2);
// 	}
// 	{
// 	  Real sign = -1.0;
// 	  Real a=Sqr(endTime);
// 	  Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
// 	  Real c=-Sqr(dx1-dx0);
// 	  PARABOLIC_RAMP_PERROR("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
// 	  Real t1,t2;
// 	  int res = quadratic(a,b,c,t1,t2);
// 	  PARABOLIC_RAMP_PERROR("Solutions: %d, %g and %g\n",res,t1,t2);
// 	}
//       }
//       if(gErrorSave) SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
//       if(gErrorGetchar) DoGetchar();
//     }
//     if(!FuzzyEquals(dx0 + a*tswitch,dx1-a*t2mT,CheckEpsilonV)) {
//       if(gVerbose >= 1) PARABOLIC_RAMP_PERROR("PPRamp::SolveMinAccel: Numerical error, v mismatch!\n");
//       if(gVerbose >= 2) {
// 	PARABOLIC_RAMP_PERROR("Velocity error %g vs %g, err %g\n",dx0+a*tswitch,dx1-a*t2mT,dx0+a*tswitch-(dx1-a*t2mT));
// 	PARABOLIC_RAMP_PERROR("ramp %g,%g -> %g, %g\n",x0,dx0,x1,dx1);
// 	PARABOLIC_RAMP_PERROR("Accel %g\n",a);
// 	PARABOLIC_RAMP_PERROR("Switch %g, total %g\n",tswitch,ttotal);
//       }
//       if(gErrorSave) SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
//       if(gErrorGetchar) DoGetchar();
//       return false;
//     }
//   }
//   return true;
// }


// //for a PP ramp:
// //xs = x0 + ts*dx0 + 0.5*z*ts^2
// //xs = x1 - (T-ts)*dx1 - 0.5*z*(T-ts)^2
// //xs' = dx0 + ts*z
// //xs' = dx1 + (T-ts)*z
// //z = sign*a
// //(2ts-T)*z = dx1 - dx0
// //ts = 1/2* (T+(dx1 - dx0)/z)
// //T-ts = 1/2* (T-(dx1 - dx0)/z)
// //2 T(dx0+dx1) + 4(x0 - x1) - (dx1 - dx0)^2/z + z*T^2 = 0
// //what if z is close to 0?
// //suppose dx1 ~= dx0, then the other solution is 
// //4 T dx0 + 4(x0 - x1) + z*T^2 = 0
// //=>z = - 4 dx0 / T + 4(x1 - x0)/T^2
// //
// //alt: let y = (dx1 - dx0)/z, z = (dx1 - dx0)/y  (y is a time shift)
// //ts = 1/2* (T+y)
// //T-ts = 1/2* (T-y)
// //x0 + 1/2(T+y)*dx0 + 0.5*z*1/4(T+y)^2 = x1 - 1/2(T-y)*dx1 - 0.5*z*1/4(T-y)^2
// //4(x0-x1) + 2(T+y)*dx0 + 0.5*z*(T+y)^2 = - 2(T-y)*dx1 - 0.5*z*(T-y)^2
// //[4(x0-x1)/T + 2(dx0+dx1)] y - y^2 (dx1 - dx0)/T + (dx1 - dx0) T = 0
// Real PPRamp::CalcMinAccel(Real endTime,Real sign,Real& switchTime) const
// {
//   Real a,b,c;
//   a = -(dx1 - dx0)/endTime;
//   b = (2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
//   c = (dx1 - dx0)*endTime;
//   Real rat1,rat2;
//   int res=quadratic(a,b,c,rat1,rat2);
//   Real accel1 = (dx1-dx0)/rat1;
//   Real accel2 = (dx1-dx0)/rat2;
//   Real switchTime1 = endTime*0.5+0.5*rat1;
//   Real switchTime2 = endTime*0.5+0.5*rat2;
//   //fix up numerical errors
//   if(switchTime1 >  endTime && switchTime1 < endTime+EpsilonT*1e-1)
//     switchTime1 = endTime;
//   if(switchTime2 >  endTime && switchTime2 < endTime+EpsilonT*1e-1)
//     switchTime2 = endTime;
//   if(switchTime1 < 0 && switchTime1 > -EpsilonT*1e-1)
//     switchTime1 = 0;
//   if(switchTime2 < 0 && switchTime2 > -EpsilonT*1e-1)
//     switchTime2 = 0;
//   if(res > 0 && FuzzyZero(rat1,EpsilonT)) {
//     //consider it as a zero, ts = T/2    
//     //z = - 4*(x0-x1)/T^2 - 2 (dx0+dx1)/T
//     accel1=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
//   }
//   if(res > 1 && FuzzyZero(rat2,EpsilonT)) {
//     accel2=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
//   }
//   bool firstInfeas = false;
//   if(res > 0 && (FuzzyZero(accel1,EpsilonA) || FuzzyZero(endTime/rat1,EpsilonA))) { //infer that accel must be small
//     //if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0!=dx1
//     //  switchTime1 = endTime*0.5;
//     //}
//     if(!FuzzyEquals(x0 + switchTime1*dx0 + 0.5*Sqr(switchTime1)*accel1,x1 - (endTime-switchTime1)*dx1 - 0.5*Sqr(endTime-switchTime1)*accel1,EpsilonX) ||
//        !FuzzyEquals(dx0+switchTime1*accel1,dx1+(endTime-switchTime1)*accel1,EpsilonV)) {
//       firstInfeas = true;
//       //LOG4CXX_INFO(KrisLibrary::logger(),"First solution "<<accel1<<" for time "<<switchTime1<<", "<<-accel1<<" for time "<<endTime-switchTime1);
//       //LOG4CXX_INFO(KrisLibrary::logger(),"First solution infeasible, x diff "<<x0 + switchTime1*dx0 + 0.5*Sqr(switchTime1)*accel1 - (x1 - (endTime-switchTime1)*dx1 - 0.5*Sqr(endTime-switchTime1)*accel1)<<", v "<<dx0+switchTime1*accel1<<" "<<(dx1+(endTime-switchTime1)*accel1));
//     }
//   }
//   if(res > 1 && (FuzzyZero(accel2,EpsilonA) || FuzzyZero(endTime/rat2,EpsilonA))) {
//     //if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0!=dx1
//     //  switchTime2 = endTime*0.5;
//     //}
//     if(!FuzzyEquals(x0 + switchTime2*dx0 + 0.5*Sqr(switchTime2)*accel2,x1 - (endTime-switchTime2)*dx1 - 0.5*Sqr(endTime-switchTime2)*accel2,EpsilonX) ||
//        !FuzzyEquals(dx0+switchTime2*accel2,dx1+(endTime-switchTime2)*accel2,EpsilonV)) {
//       //LOG4CXX_INFO(KrisLibrary::logger(),"Second solution "<<accel2<<" for time "<<switchTime2<<", "<<-accel2<<" for time "<<endTime-switchTime2);
//       //LOG4CXX_INFO(KrisLibrary::logger(),"Second solution infeasible, x "<<x0 + switchTime2*dx0 + 0.5*Sqr(switchTime2)*accel2<<" "<<x1 - (endTime-switchTime2)*dx1 - 0.5*Sqr(endTime-switchTime2)*accel2<<", v "<<dx0+switchTime2*accel2<<" "<<(dx1+(endTime-switchTime2)*accel2));
//       res--;
//     }
//   }
//   if(firstInfeas) {
//     accel1 = accel2;
//     rat1 = rat2;
//     switchTime1 = switchTime2;
//     res--;
//   }
//   if(res==0) return -1;
//   else if(res==1) {
//     if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return sign*accel1; }
//     return -1.0;
//   }
//   else if(res==2) {
//     if(switchTime1 >= 0 && switchTime1 <= endTime) {
//       if(switchTime2 >= 0 && switchTime2 <= endTime) {
// 	if(accel1 < accel2) { switchTime=switchTime1; return sign*accel1; }
// 	else { switchTime=switchTime2; return sign*accel2; }
//       }
//       else { switchTime=switchTime1; return sign*accel1; }
//     }
//     else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return sign*accel2; }
//     return -1.0;
//   }
//   if(FuzzyZero(a,EpsilonT) && FuzzyZero(b,EpsilonT) && FuzzyZero(c,EpsilonT)) {
//     switchTime = 0.5*endTime;
//     return 0;
//   }
//   return -1.0;

//   /*
//   Real a=endTime;
//   Real b=sign*(2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
//   if(FuzzyZero(b,EpsilonX)) {
//     //need to double check for numerical instability
//     //if sign is +, this means we're switching directly to -
//     //if sign is -, this means we're switching directly to +
//     //if(sign > 0.0 && x1 > x0+dx0*endTime) return -1;
//     //else if(sign < 0.0 && x1 < x0+dx0*endTime) return -1;
//     switchTime = 0;
//     Real a=(dx1-dx0)/endTime;
//     if((sign > 0.0) == (a >= 0.0)) return -1;
//     else return Abs(a);
//   }
//   Real c=-Sqr(dx1-dx0)/endTime;
//   if(FuzzyEquals(dx1,dx0,EpsilonV)) {
//     //one of the solutions will be very close to zero, use alt solution
//     Real a=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
//     LOG4CXX_INFO(KrisLibrary::logger(),"only two solutions: 0 and "<<a);
//     switchTime = 0.5*endTime;
//     //try out the zero solution
//     LOG4CXX_INFO(KrisLibrary::logger(),"diff at 0 solution: "<<x0-x1 + switchTime*(dx0+dx1));
//     if(FuzzyEquals(x0 + switchTime*dx0,x1 - switchTime*dx1,EpsilonX)) 
//       return 0;
//     PARABOLIC_RAMP_ASSERT(FuzzyEquals(dx0 + switchTime*a,dx1 + switchTime*a,CheckEpsilonV));
//     PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + switchTime*dx0 + 0.5*a*Sqr(switchTime),x1 - switchTime*dx1-0.5*a*Sqr(switchTime),CheckEpsilonX));
//     if((sign > 0.0) != (a >= 0.0)) return -1;
//     return Abs(a);
//   }
//   if(FuzzyZero(c,EpsilonA)) {
//     //need better numerical performance when dx1 ~= dx0
//     a = a/Abs(dx1-dx0);
//     b = b/Abs(dx1-dx0);
//     c = -Abs(dx1-dx0)/endTime;
//   }
//   Real accel1,accel2;
//   int res=quadratic(a,b,c,accel1,accel2);
//   //remove negative accelerations
//   if(res >= 1 && accel1 < 0) {
//     accel1 = accel2;
//     res--;
//   }
//   if(res >= 2 && accel2 < 0) {
//     res--;
//   }

//   Real switchTime1 = endTime*0.5+sign*0.5*(dx1-dx0)/accel1;
//   Real switchTime2 = endTime*0.5+sign*0.5*(dx1-dx0)/accel2;
//   //if(accel1 == 0 && x0 == x1) switchTime1 = 0;
//   //if(accel2 == 0 && x0 == x1) switchTime2 = 0;
//   if(res==0) return -1;
//   else if(res==1) {
//     if(!IsFinite(accel1)) {
//       LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing accelerations!\n");
//       LOG4CXX_INFO(KrisLibrary::logger(),"Quadratic "<<a<<" x^2 + "<<b<<" x + "<<c);
//       LOG4CXX_INFO(KrisLibrary::logger(),"x0 "<<x0<<", dx0 "<<dx0<<", x1 "<<x1<<", dx1 "<<dx1);
//       LOG4CXX_INFO(KrisLibrary::logger(),"EndTime "<<endTime<<", sign "<<sign);
//       LOG4CXX_INFO(KrisLibrary::logger(),"Results "<<accel1<<" "<<accel2);
//       DoGetchar();
//     }
//     if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return accel1; }
//     return -1.0;
//   }
//   else if(res==2) {
//     if(!IsFinite(accel1) || !IsFinite(accel2)) {
//       LOG4CXX_ERROR(KrisLibrary::logger(),"Error computing accelerations!\n");
//       LOG4CXX_INFO(KrisLibrary::logger(),"Quadratic "<<a<<" x^2 + "<<b<<" x + "<<c);
//       LOG4CXX_INFO(KrisLibrary::logger(),"x0 "<<x0<<", dx0 "<<dx0<<", x1 "<<x1<<", dx1 "<<dx1);
//       LOG4CXX_INFO(KrisLibrary::logger(),"EndTime "<<endTime<<", sign "<<sign);
//       LOG4CXX_INFO(KrisLibrary::logger(),"Results "<<accel1<<" "<<accel2);
//       DoGetchar();
//     }
//     if(FuzzyZero(switchTime1,EpsilonT) || FuzzyZero(switchTime2,EpsilonT)) {
//       //need to double check for numerical instability
//       //if sign is +, this means we're switching directly to -
//       //if sign is -, this means we're switching directly to +
//       if(sign > 0.0 && x1 > x0+dx0*endTime) return -1;
//       else if(sign < 0 && x1 < x0+dx0*endTime) return -1;
//       switchTime = 0;
//       if(FuzzyZero(switchTime1,EpsilonT)) return accel1;
//       else return accel2;
//     }
//     if(switchTime1 >= 0 && switchTime1 <= endTime) {
//       if(switchTime2 >= 0 && switchTime2 <= endTime) {
// 	if(switchTime1 < switchTime2) { switchTime=switchTime1; return accel1; }
// 	else { switchTime=switchTime2; return accel2; }
//       }
//       else { switchTime=switchTime1; return accel1; }
//     }
//     else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return accel2; }
//     return -1.0;
//   }
//   return -1.0;
//   */
// }



// }
