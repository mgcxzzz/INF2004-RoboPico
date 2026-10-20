#include <assert.h>
#include <stdio.h>
#include "logic.h"
#include "integration.h"
#include "s3_line.h"
#include "s3_config.h"
#include "log_store.h"
int main(void) {
    BenchArm arm={0};
    assert(!bench_arm_step(&arm,100,0,0,1,1));
    assert(!bench_arm_step(&arm,2999999,0,0,1,1));assert(arm.state==ARM_HOLD_BOTH);
    assert(!bench_arm_step(&arm,3000100,0,0,1,1));assert(arm.state==ARM_RELEASE_BOTH);
    /* Either release order is accepted; a fresh tap and release is mandatory. */
    assert(!bench_arm_step(&arm,3100000,0,0,0,1));assert(arm.state==ARM_RELEASE_BOTH);
    assert(!bench_arm_step(&arm,3200000,0,0,0,0));assert(arm.state==ARM_TAP);
    assert(!bench_arm_step(&arm,3500000,0,0,1,0));assert(arm.state==ARM_TAP_RELEASE);
    assert(!bench_arm_step(&arm,4000000,0,0,0,0));assert(arm.state==ARM_COUNTDOWN);
    assert(!bench_arm_step(&arm,5999999,0,0,0,0));
    assert(bench_arm_step(&arm,6000000,0,0,0,0));
    assert(!bench_arm_step(&arm,6100000,0,0,0,0)); /* no repeated start */
    /* Cancellation can be rearmed without a power cycle. */
    arm=(BenchArm){ARM_COUNTDOWN,0};
    assert(!bench_arm_step(&arm,1000000,0,0,0,1));assert(arm.state==ARM_IDLE);
    assert(!bench_arm_step(&arm,2000000,0,0,1,1));
    assert(!bench_arm_step(&arm,5000000,0,0,1,1));assert(arm.state==ARM_RELEASE_BOTH);
    assert(!bench_arm_step(&arm,5100000,0,0,1,0));
    assert(!bench_arm_step(&arm,5200000,0,0,0,0));
    assert(!bench_arm_step(&arm,5500000,0,0,1,0));
    assert(!bench_arm_step(&arm,5600000,0,0,0,0));
    assert(bench_arm_step(&arm,7600000,0,0,0,0));
    arm=(BenchArm){ARM_COUNTDOWN,0};
    assert(!bench_arm_step(&arm,3000000,1,0,0,0));assert(arm.state==ARM_IDLE);
    arm=(BenchArm){ARM_COUNTDOWN,0};
    assert(!bench_arm_step(&arm,3000000,0,1,0,0));assert(arm.state==ARM_IDLE);
    /* Unsigned elapsed timing remains valid across the 32-bit timer wrap. */
    arm=(BenchArm){ARM_COUNTDOWN,0xffff0000u};
    assert(bench_arm_step(&arm,2000000u,0,0,0,0));
    s3_line_t line; s3_line_out_t proposal;
    int16_t target[2];
    s3_line_reset(&line);
    s3_line_step(&line,0,0,&proposal);
    assert(bench_targets(proposal.left_mm_s,proposal.right_mm_s,target));
    assert(target[0]==75 && target[1]==75);
    s3_line_step(&line,S3_SENSE_LEFT,1,&proposal);
    assert(bench_targets(proposal.left_mm_s,proposal.right_mm_s,target));
    assert(target[0]==30 && target[1]==95);
    s3_line_step(&line,S3_SENSE_RIGHT,1,&proposal);
    assert(bench_targets(proposal.left_mm_s,proposal.right_mm_s,target));
    assert(target[0]==95 && target[1]==30);
    s3_line_step(&line,0,1,&proposal);
    assert(bench_targets(proposal.left_mm_s,proposal.right_mm_s,target));
    assert(target[0]==82 && target[1]==62); /* Original memory-based correction. */
    s3_line_step(&line,S3_SENSE_LEFT|S3_SENSE_RIGHT,12,&proposal);
    assert(proposal.events&S3_LEV_JUNCTION);
    s3_line_step(&line,0,250,&proposal);
    assert(proposal.events&S3_LEV_LOST);
    assert(!bench_targets(-1,100,target));assert(!bench_targets(201,100,target));
    assert(bench_stale(50000,0));assert(!bench_stale(49999,0));
    assert(bench_stale(10000,0xffff0000u));
    assert(abs_ticks(-2147483647-1)==2147483648u);
    assert(sizeof(BenchLog)<=4096 && BENCH_ROWS>=62);
    assert(log_crc("123456789",9)==0xcbf43926u);
    puts("PASS: offline hold/release/tap/countdown, cancel/retry, USB/busy refusal, clock wrap; real S3 targets; log bounds/CRC");
}
