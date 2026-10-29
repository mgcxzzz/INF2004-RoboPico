#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "continuous_steering.h"
#undef CONTINUOUS_STEERING_H
#define ContinuousSteering ReferenceSteering
#define continuous_steering_reset reference_reset
#define continuous_steering_finite reference_finite
#define continuous_steering_step reference_step
#include "continuous_steering_reference.h"
#undef ContinuousSteering
#undef continuous_steering_reset
#undef continuous_steering_finite
#undef continuous_steering_step
int main(int argc,char **argv) {
    assert(argc==2);FILE *f=fopen(argv[1],"r");assert(f);
    char line[512];assert(fgets(line,sizeof(line),f));
    ContinuousSteering raised={0};ReferenceSteering old={0};
    Circuit a={.phase=C_FOLLOW},b={.phase=C_FOLLOW};
    AnalogDemand previous_a={0},previous_b={0};unsigned rows=0;
    double variation_a=0,variation_b=0,mean_a=0,mean_b=0;
    int min_a=999,min_b=999;
    while(fgets(line,sizeof(line),f)) {
        unsigned us,dt,level_l,level_r,profile,flags;int opt_l,opt_r;float speed_l,speed_r;
        assert(sscanf(line,"%u,%u,%d,%d,%u,%u,%f,%f,%u,%u",&us,&dt,&opt_l,&opt_r,
            &level_l,&level_r,&speed_l,&speed_r,&profile,&flags)==10);
        AnalogDemand d={.target={(int16_t)opt_l,(int16_t)opt_r},
            .level={(uint16_t)level_l,(uint16_t)level_r},
            .decision=opt_l<opt_r?SYM_LEFT:opt_l>opt_r?SYM_RIGHT:SYM_STRAIGHT};
        CircuitInput i={.now=us,.speed={speed_l,speed_r}};
        AnalogDemand x=continuous_steering_step(&raised,d,&a,&i,(flags&4)!=0,profile,dt/1000000.0f);
        AnalogDemand y=reference_step(&old,d,&b,&i,(flags&4)!=0,profile,dt/1000000.0f);
        assert(x.target[0]>=0 && x.target[1]>=0 && x.target[0]<=45 && x.target[1]<=45);
        if(rows)for(unsigned w=0;w<2;++w){variation_a+=abs(x.target[w]-previous_a.target[w]);variation_b+=abs(y.target[w]-previous_b.target[w]);}
        mean_a+=(x.target[0]+x.target[1])*.5;mean_b+=(y.target[0]+y.target[1])*.5;
        for(unsigned w=0;w<2;++w){if(x.target[w]<min_a)min_a=x.target[w];if(y.target[w]<min_b)min_b=y.target[w];}
        previous_a=x;previous_b=y;++rows;
    }
    fclose(f);assert(rows==256);
    printf("FIXED-INPUT REPLAY: rows=%u target_total_variation old=%.0f height=%.0f; mean_target old=%.2f height=%.2f; minimum_wheel_target old=%d height=%d\n",
        rows,variation_b,variation_a,mean_b/rows,mean_a/rows,min_b,min_a);
    puts("This compares emitted targets for identical recorded sensor/speed inputs. It does not predict physical tracking, barcode success or bend performance.");
}
