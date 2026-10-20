#include <assert.h>
#include <stdio.h>
#include "logic.h"
#include "integration.h"
#include "s3_line.h"
#include "s3_config.h"
#include "log_store.h"
int main(void) {
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
    puts("PASS: real S3 left/right/centred/remembered steering -> scaled motor targets; events; stale wrap; log bounds/CRC");
}
