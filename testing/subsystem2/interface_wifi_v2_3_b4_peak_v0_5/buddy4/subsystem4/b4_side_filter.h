/* Robust per-FIFO gravity for side height only. Raw samples still feed shocks. */
#ifndef B4_SIDE_FILTER_H
#define B4_SIDE_FILTER_H
#include "imu_proc.h"
static inline int32_t b4_median(int32_t *v, unsigned n) {
    for(unsigned i=1;i<n;i++) {
        int32_t x=v[i];unsigned j=i;
        while(j && v[j-1]>x) {v[j]=v[j-1];j--;}
        v[j]=x;
    }
    return n&1U?v[n/2U]:(v[n/2U-1U]+v[n/2U])/2;
}
static inline bool b4_side_filter(b4_vec_t const *samples, unsigned n,
                                  int16_t gravity[3], uint16_t *max_mad,
                                  uint16_t axis_mad[3]) {
    int32_t v[B4_FIFO_DEPTH];
    *max_mad=0;
    axis_mad[0]=axis_mad[1]=axis_mad[2]=0;
    if(n<3U || n>B4_FIFO_DEPTH) return false;
    for(unsigned axis=0;axis<3;axis++) {
        for(unsigned i=0;i<n;i++) v[i]=axis==0?samples[i].fwd_mg:
                                             axis==1?samples[i].lat_mg:samples[i].up_mg;
        int32_t median=b4_median(v,n);
        gravity[axis]=(int16_t)median;
        for(unsigned i=0;i<n;i++) {int32_t d=v[i]-median;v[i]=d<0?-d:d;}
        uint16_t mad=(uint16_t)b4_median(v,n);
        axis_mad[axis]=mad; /* v0.6 diagnostic: 0 fwd, 1 lat, 2 up */
        if(mad>*max_mad)*max_mad=mad;
    }
    return true;
}
#endif
