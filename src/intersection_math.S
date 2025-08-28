.text
.syntax unified

# most of the circle math involved
# after the registers have the following layout:
#
# q2-q5 = free registers
# q6-q9 = sqrt(g)
# q10-q13 = dot products
.macro mainCircleRoutine
    vldmia r1!, {q2-q5} @# x
    vldmia r2!, {q6-q9} @# y

    vsub.f32 q2, q14, q2 @# dx = ox - x
    vsub.f32 q6, q15, q6 @# dy = oy - y
                        
    vsub.f32 q3, q14, q3 @# dx = ox - x 
    vsub.f32 q7, q15, q7 @# dy = oy - y 
                        
    vsub.f32 q4, q14, q4 @# dx = ox - x 
    vsub.f32 q8, q15, q8 @# dy = oy - y 
                        
    vsub.f32 q5, q14, q5 @# dx = ox - x 
    vsub.f32 q9, q15, q9 @# dy = oy - y 

    vmul.f32 q10, q2, d0[0] @# partial_dot = dx * vx
    vmul.f32 q11, q3, d0[0] @# partial_dot = dx * vx
    vmul.f32 q12, q4, d0[0] @# partial_dot = dx * vx
    vmul.f32 q13, q5, d0[0] @# partial_dot = dx * vx

# using vmla here is fine since its result is not needed for a while
    vmla.f32 q10, q6, d0[1] @# dot = partial_dot + dy * vy
    vmla.f32 q11, q7, d0[1] @# dot = partial_dot + dy * vy
    vmla.f32 q12, q8, d0[1] @# dot = partial_dot + dy * vy
    vmla.f32 q13, q9, d0[1] @# dot = partial_dot + dy * vy

# calculate squared cross products:
# cross = (dx * vy - dy * vx) ^ 2

# here the d registers get freed, so they can be re-used for cross product here
# since the dy registers are free now we can use them store the intermediate result

    vmul.f32 q2, q2, d0[1] @# partial_cross = dx * vy
    vmul.f32 q6, q6, d0[0] @# dy * vx

    vmul.f32 q3, q3, d0[1] @# partial_cross = dx * vy
    vmul.f32 q7, q7, d0[0] @# dy * vx
    
    vmul.f32 q4, q4, d0[1] @# partial_cross = dx * vy
    vmul.f32 q8, q8, d0[0] @# dy * vx

    vmul.f32 q5, q5, d0[1] @# partial_cross = dx * vy
    vmul.f32 q9, q9, d0[0] @# dy * vx

    vsub.f32 q2, q2, q6 @# cross = partial_cross - dy * vx
    vsub.f32 q3, q3, q7 @# cross = partial_cross - dy * vx
    vsub.f32 q4, q4, q8 @# cross = partial_cross - dy * vx
    vsub.f32 q5, q5, q9 @# cross = partial_cross - dy * vx

    vmul.f32 q2, q2, q2 @# cross^2 = cross * cross
    vmul.f32 q3, q3, q3 @# cross^2 = cross * cross
    vmul.f32 q4, q4, q4 @# cross^2 = cross * cross
    vmul.f32 q5, q5, q5 @# cross^2 = cross * cross

# now q6-q9 can be reused for other purposes

# now calculate the g = r^2 - cross^2
    vsub.f32 q6, q1, q2
    vsub.f32 q7, q1, q3
    vsub.f32 q8, q1, q4
    vsub.f32 q9, q1, q5

# calculate inverse square root of g
    vrsqrte.f32 q2, q6 @# 1/sqrt(g)
    vrsqrte.f32 q3, q7 @# 1/sqrt(g)
    vrsqrte.f32 q4, q8 @# 1/sqrt(g)
    vrsqrte.f32 q5, q9 @# 1/sqrt(g)

# refine sqrt estimate - precision is good enough without 
# we don't have enough registers to peform the refine step, so we save them temporarily to the stack
    # vpush {q10-q13}

# 1/(estimate of sqrt(g))^2
    # vmul.f32 q10, q2, q2 
    # vmul.f32 q11, q3, q3 
    # vmul.f32 q12, q4, q4 
    # vmul.f32 q13, q5, q5 
    #
    # vrsqrts.f32 q10, q10, q6
    # vrsqrts.f32 q11, q11, q7
    # vrsqrts.f32 q12, q12, q8
    # vrsqrts.f32 q13, q13, q9
    #
    # vmul.f32 q2, q10, q2
    # vmul.f32 q3, q11, q3
    # vmul.f32 q4, q12, q4
    # vmul.f32 q5, q13, q5
    #
    # vpop {q10-q13}

# multiply the inverse by the original to get the final sqrt:
    vmul.f32 q6, q2, q6
    vmul.f32 q7, q3, q7
    vmul.f32 q8, q4, q8
    vmul.f32 q9, q5, q9
.endm


.global circleIntersection
.p2align 4
.type circleIntersection, %function
.thumb
.thumb_func
circleIntersection:

# void circleIntersection(
# r0 = float* res,
# r1 = float* x,
# r2 = float* y,
#
# r3 = uint32_t len,
#
# s0 = float vx,
# s1 = float vy,
#
# s2 = float ox,
# s3 = float oy,
# s4 = float r
# );

# registers:
# dx -> squared cross ->   ->         -> res
# dy ->               -> g ->  sqrt_g -> 
# dots                                ->     -> mask-> res

# active registers:
# q2 - q5  = dx / squared cross / sqrt_g / infinity
# q6 - q9 = dy / g / mask
# q10 - q13 = dots / res / res

# static registers - don't get changed across loops:
# dx,dy = q0
# r^2 = q1
# ox = q14
# dy = q15

# precomputations get performed here

# save registers
vpush {q4-q7}

# calculate r^2
vmul.f32 d2, d2, d2

# store into q1 register
vdup.f32 q1, d2[0]

# store ox and oy
vdup.f32 q14, d1[0] // ox
vdup.f32 q15, d1[1] // oy

# main loop logic
.CircleIntersectionLoop:
# executes most of the routine
    mainCircleRoutine

    vsub.f32 q2, q10, q6 @# res = dot - sqrt(g)
    vsub.f32 q3, q11, q7 @# res = dot - sqrt(g)
    vsub.f32 q4, q12, q8 @# res = dot - sqrt(g)
    vsub.f32 q5, q13, q9 @# res = dot - sqrt(g)

# check if res is valid

# creates the mask - compare with zero - maybe needs more checks
    vcge.f32 q10, q2, #0  @# mask = res >= 0
    vcge.f32 q11, q3, #0  @# mask = res >= 0
    vcge.f32 q12, q4, #0  @# mask = res >= 0
    vcge.f32 q13, q5, #0  @# mask = res >= 0

# loads roughly 8.589935 * 10^9 into the q6 registers
    vmov.i32 q6, 0x50000000

# mask res with infinity/big number
    vbsl.f32 q10, q2, q6 @# mask ? res : infinity
    vbsl.f32 q11, q3, q6 @# mask ? res : infinity
    vbsl.f32 q12, q4, q6 @# mask ? res : infinity
    vbsl.f32 q13, q5, q6 @# mask ? res : infinity

# decrement len, set flags
    subs r3, r3, #16

# res is now computed, just need to store the result
    vstmia r0!, {q10-q13}

# other part of loop
    bgt .CircleIntersectionLoop

# end of function
vpop {q4-q7}
bx lr

.global circleCenterIntersection 
.p2align 4
.type circleCenterIntersection, %function
.thumb
.thumb_func
circleCenterIntersection:
# precomputations get performed here

# save registers
vpush {q4-q7}

# calculate r^2
vmul.f32 d2, d2, d2

# store into q1 register
vdup.f32 q1, d2[0]

# store ox and oy
vdup.f32 q14, d1[0] // ox
vdup.f32 q15, d1[1] // oy

# main loop logic
.CircleCenterLoop:
    mainCircleRoutine

# keeps original dot intact, as that is the final returned result
    vsub.f32 q2, q10, q6 @# res = dot - sqrt(g)
    vsub.f32 q3, q11, q7 @# res = dot - sqrt(g)
    vsub.f32 q4, q12, q8 @# res = dot - sqrt(g)
    vsub.f32 q5, q13, q9 @# res = dot - sqrt(g)

# check if res is valid

# creates the mask - compare with zero - maybe needs more checks
    vcge.f32 q6, q2, #0  @# mask = res >= 0
    vcge.f32 q7, q3, #0  @# mask = res >= 0
    vcge.f32 q8, q4, #0  @# mask = res >= 0
    vcge.f32 q9, q5, #0  @# mask = res >= 0

# loads roughly 8.589935 * 10^9
    vmov.i32 q2, 0x50000000

# mask res with infinity/big number
    vbsl.f32 q6, q10, q2 @# mask ? dot : infinity
    vbsl.f32 q7, q11, q2 @# mask ? dot : infinity
    vbsl.f32 q8, q12, q2 @# mask ? dot : infinity
    vbsl.f32 q9, q13, q2 @# mask ? dot : infinity

# decrement len, set flags
    subs r3, r3, #16

# res is now computed, just need to store the result
    vstm r0!, {q6-q9}

# other part of loop
    bgt .CircleCenterLoop

# end of function
vpop {q4-q7}
bx lr




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
# q14 - infinity

# this leaves 12 registers free

vdup.f32 q15, d2[1] // moves sc, done first so next instruction doesn't override current
vdup.f32 q1, d2[0]  // moves rc

# loads roughly 8.589935 * 10^9
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
    vcle.f32 q2, q2, #1  // s =< 1
    vcle.f32 q3, q3, #1  // s =< 1
    vcle.f32 q4, q4, #1  // s =< 1
    vcle.f32 q5, q5, #1  // s =< 1

    // && both into one register
    vand.f32 q2, q2, q6
    vand.f32 q3, q3, q7
    vand.f32 q4, q4, q8
    vand.f32 q5, q5, q9

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
anything
# other part of loop
    bgt .RaySegmentLoop

# end of function
vpop {q4-q7}
bx lr
