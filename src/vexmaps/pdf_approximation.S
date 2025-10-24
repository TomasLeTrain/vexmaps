.text
.syntax unified

# function signature:
# inline void asm_pdfApproximation(float* res,
#                                  float* x,
#                                  uint32_t n,
#                                  float C0,
#                                  float C1,
#                                  float C2,
#                                  float C3);


// routine that can get reused by exponential / normal distribution.
// len should either be divisible by two or be large enough to fit extra numbers

.global asm_pdfApproximation 
.p2align 4
.type asm_pdfApproximation, %function
.thumb
.thumb_func
asm_pdfApproximation:
# precomputations get performed here

# save registers
vpush {q4-q7}

# r0 -> res
# r1 -> x
# r2 -> len

# q0
# d0[0] -> c0
# d0[1] -> c1
# d1[0] -> c2
# d1[1] -> c3

# main loop uses q0 through q11

# q12 -> c0
# q13 -> c1
# q14 -> c2
# q15 -> c3

vdup.f32 q12, d0[0]  // c0
vdup.f32 q13, d0[1]  // c1
vdup.f32 q14, d1[0]  // c2
vdup.f32 q15, d1[1]  // c3

.pdfApproximationLoop:
	vldmia r1!, {q0-q5}
	// q0-q7 are now loaded

	// x2 = x + c0
	vadd.f32 q0, q0, q12
	vadd.f32 q1, q1, q12
	vadd.f32 q2, q2, q12
	vadd.f32 q3, q3, q12
	vadd.f32 q4, q4, q12
	vadd.f32 q5, q5, q12

	// x^2 = (x+c0)^2
	vmul.f32 q0, q0, q0
	vmul.f32 q1, q1, q1
	vmul.f32 q2, q2, q2
	vmul.f32 q3, q3, q3
	vmul.f32 q4, q4, q4
	vmul.f32 q5, q5, q5

	// t1 = x^2 * c3
	vmul.f32 q6,  q0, q15
	vmul.f32 q7,  q1, q15
	vmul.f32 q8,  q2, q15
	vmul.f32 q9,  q3, q15
	vmul.f32 q10, q4, q15
	vmul.f32 q11, q5, q15

	// t1 = t1 + c2
	vadd.f32 q6,  q6,  q14
	vadd.f32 q7,  q7,  q14
	vadd.f32 q8,  q8,  q14
	vadd.f32 q9,  q9,  q14
	vadd.f32 q10, q10, q14
	vadd.f32 q11, q11, q14

	// t2 = x^2 * t1
	vmul.f32 q6,  q0, q6 
	vmul.f32 q7,  q1, q7 
	vmul.f32 q8,  q2, q8 
	vmul.f32 q9,  q3, q9 
	vmul.f32 q10, q4, q10
	vmul.f32 q11, q5, q11

	// t2 = c1 + t2
	vadd.f32 q0, q6,  q13
	vadd.f32 q1, q7,  q13
	vadd.f32 q2, q8,  q13
	vadd.f32 q3, q9,  q13
	vadd.f32 q4, q10, q13
	vadd.f32 q5, q11, q13

	// here the q6-q11 register get freed, so we can use them to store res
	vldmia r0, {q6-q11}

	// pdf(x) = 1 / t2
	vrecpe.f32 q0, q0
	vrecpe.f32 q1, q1
	vrecpe.f32 q2, q2
	vrecpe.f32 q3, q3
	vrecpe.f32 q4, q4
	vrecpe.f32 q5, q5

	// res += pdf(x)
	vadd.f32 q0, q0, q6 
	vadd.f32 q1, q1, q7 
	vadd.f32 q2, q2, q8 
	vadd.f32 q3, q3, q9 
	vadd.f32 q4, q4, q10
	vadd.f32 q5, q5, q11

	# decrement len, set flags
    subs r2, r2, #24

	# store back to res
    vstm r0!, {q0-q5}

	# other part of loop
    bgt .pdfApproximationLoop

# end of function
vpop {q4-q7}
bx lr
