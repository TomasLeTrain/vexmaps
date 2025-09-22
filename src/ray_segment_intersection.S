.text
.syntax unified

# has signature of:
# void asm_raySegmentDistance(float* res,
#                                        float* x,
#                                        float* y,
#
#                                        uint32_t len,
#
#                                        float ra,
#                                        float sa,
#
#                                        float sb,
#                                        float rb,
#
#                                        float rc,
#                                        float sc);

.global asm_raySegmentDistance 
.p2align 4
.type asm_raySegmentDistance, %function
.thumb
.thumb_func
asm_raySegmentDistance:
# precomputations get performed here

# save registers
vpush {q4-q7}

# q0
# d0[0] -> ra
# d0[1] -> sa
# d1[0] -> rb
# d1[1] -> sb

# q1
# d2[0] -> rc
# d2[1] -> sc

# rc and sc must be loaded into their own registers
# the others are multipliers so they can just all be in one register

# q0 - factors
# q1 - rc
# q15 - sc
# q14 - 1 / infinity

# this leaves 12 registers free

vdup.f32 q15, d2[1] // moves sc, done first so next instruction doesn't override current
vdup.f32 q1, d2[0]  // moves rc

vmov.i32 q14, 0x50000000

# main loop logic
.RaySegmentLoop:
    vldmia r1!, {q2-q5} @# x
    vldmia r2!, {q6-q9} @# y

    # create r and s

    // r = ra * y - rb * x - rc
    vmul.f32 q10, q6, d0[0] // y * ra
    vmul.f32 q11, q7, d0[0] // y * ra
    vmul.f32 q12, q8, d0[0] // y * ra
    vmul.f32 q13, q9, d0[0] // y * ra
    
    vmls.f32 q10, q2, d1[0] // y * ra - x * rb
    vmls.f32 q11, q3, d1[0] // y * ra - x * rb
    vmls.f32 q12, q4, d1[0] // y * ra - x * rb
    vmls.f32 q13, q5, d1[0] // y * ra - x * rb

    vsub.f32 q10, q10, q1 // y * ra - x * rb - rc
    vsub.f32 q11, q11, q1 // y * ra - x * rb - rc
    vsub.f32 q12, q12, q1 // y * ra - x * rb - rc
    vsub.f32 q13, q13, q1 // y * ra - x * rb - rc

    // now create s
    // s = sa * y - sb * x - sc
    // x and y registers get freed, so we can avoid using vmls

    vmul.f32 q6, q6, d0[1] // y * sa
    vmul.f32 q2, q2, d1[1] // x * sb

    vmul.f32 q7, q7, d0[1] // y * sa
    vmul.f32 q3, q3, d1[1] // x * sb

    vmul.f32 q8, q8, d0[1] // y * sa
    vmul.f32 q4, q4, d1[1] // x * sb

    vmul.f32 q9, q9, d0[1] // y * sa
    vmul.f32 q5, q5, d1[1] // x * sb

    vsub.f32 q2, q6, q2 // y * sa - x * sb
    vsub.f32 q3, q7, q3 // y * sa - x * sb
    vsub.f32 q4, q8, q4 // y * sa - x * sb
    vsub.f32 q5, q9, q5 // y * sa - x * sb

	// moves 1.0 into q14
	// done far back enough to where latency isn't an issue
	vmov.f32 q14, 1.0

    vsub.f32 q2, q2, q15 // s = y * sa - x * sb - sc
    vsub.f32 q3, q3, q15 // s = y * sa - x * sb - sc
    vsub.f32 q4, q4, q15 // s = y * sa - x * sb - sc
    vsub.f32 q5, q5, q15 // s = y * sa - x * sb - sc

    // here q6-q9 are free - get used to hold the mask
    vcge.f32 q6, q2, #0  // s >= 0
    vcge.f32 q7, q3, #0  // s >= 0
    vcge.f32 q8, q4, #0  // s >= 0
    vcge.f32 q9, q5, #0  // s >= 0

    // since we don't have enough registers left, we override s since this is the last check used by s
	
    vcle.f32 q2, q2, q14   // s <= 1
    vcle.f32 q3, q3, q14   // s <= 1
    vcle.f32 q4, q4, q14   // s <= 1
    vcle.f32 q5, q5, q14   // s <= 1

    // && both into one register
    vand.f32 q2, q2, q6
    vand.f32 q3, q3, q7
    vand.f32 q4, q4, q8
    vand.f32 q5, q5, q9

	// loads inifinity into q14 (roughly 8.589935 * 10^9)
	vmov.i32 q14, 0x50000000

    vcge.f32 q6, q10, #0  // r >= 0
    vcge.f32 q7, q11, #0  // r >= 0
    vcge.f32 q8, q12, #0  // r >= 0
    vcge.f32 q9, q13, #0  // r >= 0

    // && to the current mask
    vand.f32 q2, q2, q6
    vand.f32 q3, q3, q7
    vand.f32 q4, q4, q8
    vand.f32 q5, q5, q9

# mask res with infinity/big number
    vbsl.f32 q2, q10, q14 @# mask ? res : infinity
    vbsl.f32 q3, q11, q14 @# mask ? res : infinity
    vbsl.f32 q4, q12, q14 @# mask ? res : infinity
    vbsl.f32 q5, q13, q14 @# mask ? res : infinity

# decrement len, set flags
    subs r3, r3, #16

# res is now computed, just need to store the result
    vstm r0!, {q2-q5}

# other part of loop
    bgt .RaySegmentLoop

# end of function
vpop {q4-q7}
bx lr
