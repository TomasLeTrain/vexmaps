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

