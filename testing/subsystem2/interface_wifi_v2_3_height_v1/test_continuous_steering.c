#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "continuous_steering.h"
static AnalogDemand demand(int l,int r,unsigned a,unsigned b,unsigned decision) {
 AnalogDemand d={.target={(int16_t)l,(int16_t)r},.level={(uint16_t)a,(uint16_t)b},.decision=(uint8_t)decision};return d;
}
static Circuit following(void){Circuit c={0};c.phase=C_FOLLOW;return c;}
static CircuitInput speeds(float l,float r){CircuitInput i={0};i.speed[0]=l;i.speed[1]=r;return i;}
static AnalogDemand step(ContinuousSteering *s,AnalogDemand d,Circuit *c,CircuitInput *i) {
 return continuous_steering_step(s,d,c,i,true,1,.01f);
}
static void brief_and_sustained(void) {
 Circuit c=following();CircuitInput i=speeds(35,35);ContinuousSteering s={0};
 AnalogDemand l=demand(0,45,900,100,SYM_LEFT),r=demand(45,0,100,900,SYM_RIGHT),o;
 o=step(&s,l,&c,&i);assert(fabsf(s.gain-.45f)<.001f);
 assert(o.target[0]>=0 && o.target[1]<=45);
 for(unsigned k=0;k<200;++k){o=step(&s,k%20<10?l:r,&c,&i);if(k>20){
  assert((o.target[0]+o.target[1])*.5f>=30);assert(o.target[0]>=15 && o.target[1]>=15);
 }}
 for(unsigned k=0;k<100;++k)o=step(&s,l,&c,&i);
 assert(s.gain==1 && s.same_side_duration>=.45f);
 assert(o.target[0]==0 && o.target[1]==45);
 /* Reverse requests take effect without a timed steering lock. */
 for(unsigned k=0;k<15;++k)o=step(&s,r,&c,&i);
 assert(o.target[0]>o.target[1]);
}
static void continuous_excess_yaw_damping(void) {
 Circuit ca=following(),cb=following();ContinuousSteering a={0},b={0};
 CircuitInput turning=speeds(25,55),still=speeds(40,40);
 AnalogDemand d=demand(40,50,700,300,SYM_LEFT),x,y;
 for(unsigned k=0;k<20;++k){x=step(&a,d,&ca,&turning);y=step(&b,d,&cb,&still);}
 assert(a.damp_weight>0 && b.damp_weight==0);
 assert((x.target[1]-x.target[0])<(y.target[1]-y.target[0]));
 /* A true bend at its requested wheel differential retains full authority. */
 a=(ContinuousSteering){0};ca=following();turning=speeds(0,45);
 for(unsigned k=0;k<100;++k)x=step(&a,demand(0,45,900,100,SYM_LEFT),&ca,&turning);
 assert(a.gain==1 && a.damp_weight==0 && x.target[0]==0 && x.target[1]==45);
 /* No optical turn request: encoder rotation is still damped. */
 x=step(&a,demand(45,45,600,590,SYM_STRAIGHT),&ca,&turning);
 for(unsigned k=0;k<15;++k)x=step(&a,demand(45,45,600,590,SYM_STRAIGHT),&ca,&turning);
 assert(a.damp_weight>0 && x.target[0]>x.target[1]);
}
static void symmetry_and_brightness(void) {
 Circuit ca=following(),cb=following(),cc=following();ContinuousSteering a={0},b={0},c={0};
 CircuitInput ia=speeds(35,45),ib=speeds(45,35);AnalogDemand x,y,z;
 for(unsigned k=0;k<200;++k){
  bool sign=(k/17)%2;AnalogDemand da=demand(sign?45:10,sign?10:45,sign?200:800,sign?800:200,sign?SYM_RIGHT:SYM_LEFT);
  AnalogDemand db=da;db.target[0]=da.target[1];db.target[1]=da.target[0];db.level[0]=da.level[1];db.level[1]=da.level[0];
  AnalogDemand dc=da;dc.level[0]+=100;dc.level[1]+=100;
  x=step(&a,da,&ca,&ia);y=step(&b,db,&cb,&ib);z=step(&c,dc,&cc,&ia);
  assert(abs(x.target[0]-y.target[1])<=1 && abs(x.target[1]-y.target[0])<=1);
  assert(x.target[0]==z.target[0] && x.target[1]==z.target[1]);
 }
}
static void bypass_and_limits(void) {
 Circuit c=following();ContinuousSteering s={.active=1,.mean=40,.half=10};CircuitInput i=speeds(40,40);
 AnalogDemand d=demand(0,45,0,0,SYM_RECOVER_LEFT),o;
 c.follower.core.white=true;o=step(&s,d,&c,&i);assert(!s.active && o.target[0]==0 && o.target[1]==45);
 c=following();s.active=1;d=demand(45,45,500,500,SYM_STRAIGHT);d.stop=true;
 o=step(&s,d,&c,&i);assert(o.stop && !s.active);
 s.active=1;d=demand(0,0,500,500,SYM_STRAIGHT);o=step(&s,d,&c,&i);assert(!s.active && !o.target[0] && !o.target[1]);
 for(unsigned phase=0;phase<15;++phase)if(phase!=C_FOLLOW){c.phase=phase;s.active=1;d=demand(-30,60,800,200,SYM_LEFT);
  o=step(&s,d,&c,&i);assert(!s.active && o.target[0]==-30 && o.target[1]==60);}
 c=following();d=demand(135,135,800,800,C_FAST);s=(ContinuousSteering){0};
 for(unsigned k=0;k<200;++k)o=continuous_steering_step(&s,d,&c,&i,false,3,.01f);
 assert(o.target[0]==135 && o.target[1]==135 && c.fast);
 o=continuous_steering_step(&s,d,&c,&i,true,3,.01f);assert(o.target[0]<=45 && o.target[1]<=45 && !c.fast);
 s=(ContinuousSteering){0};i=speeds(NAN,NAN);o=continuous_steering_step(&s,d,&c,&i,false,1,NAN);
 assert(isfinite(s.mean) && isfinite(s.half) && o.target[0]>=0 && o.target[1]>=0);
}
int main(void){brief_and_sustained();continuous_excess_yaw_damping();symmetry_and_brightness();bypass_and_limits();
 puts("PASS raised-height steering: bounded brief corrections, persistent bends, continuous excess-yaw damping, mirrored/brightness-independent outputs, urgent bypass and profile limits.");}
