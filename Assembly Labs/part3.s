/* Program to Count the number of 1's and Zeroes in a sequence of 32-bit words,
and determines the largest of each */

.global _start
_start:

/* Your code here  */
	la t0, TEST_NUM
	la t1, LargestOnes
	lw a1, 0(t1)
	la t2, LargestZeroes
	lw a2, 0(t2)
	#li s1, 0		#store count number

loop:
#count 1's
	li s4, 32
	li s1, 0							#reset count = 0 
	li s5, 0
	lw a0, 0(t0)
	beq a0, zero, printBoard
	j count_loop
	
#count 1s in num	
count_loop:
#check 1s
	andi s0, a0, 1							#check if LSB is 1
	#xori s0, s0, 1
	add s1, s1, s0							#update number of 0/1 
#check 0s
	andi s6, a0, 1
	xor s5, s5, 1							#check if LSB is 1
	add s5, s5, s6							#update number of 0 
#shif bits to right
	srli a0, a0, 1							#shift left by 1 bits, change LSB to next bits in InputWord
	addi s4, s4, -1
	bne s4, zero, count_loop				#keep going untill read 0
	addi t0, t0, 4							#move next num in array
#update largest 1s or 0s
	bge s1, a1, update_greaterOnes				#if current num greater that previous num, update largest to current value
	bge s5, a2, update_greaterZeroes
	j loop
update_greaterOnes:
	move a1, s1
	j loop
update_greaterZeroes:
	move a2, s5
	j loop
	
                	# When t0 is zero, return from the delay function
printBoard:
	li t0, 1000     # Set a delay count 
	.equ LEDs, 0xFF200000
	la s3, LEDs
	sw a1,(s3)
	
	delay_loop:
    addi t0, t0, -1     			# Decrement the counter by 1
    bnez t0, delay_loop 
	j PrintSecondBoard
PrintSecondBoard:
	li t0, 1000
	.equ LEDs, 0xFF200000
	la s10, LEDs
	sw a2, (s10)
	
	delay_secondloop:
    addi t0, t0, -1     			# Decrement the counter by 1
    bnez t0, delay_secondloop 

	j printBoard
	
stop: j stop

.data
TEST_NUM:  .word 0x4a01fead, 0xF677D671,0xDC9758D5,0xEBBD45D2,0x8059519D
            .word 0x76D8F0D2, 0xB98C9BB5, 0xD7EC3A9E, 0xD9BADC01, 0x89B377CD
            .word 0  # end of list 

LargestOnes: .word 0
LargestZeroes: .word 0