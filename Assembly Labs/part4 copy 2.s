.global _start
_start:

# Load addresses for LEDs and the COUNT and RUN global variables
la s0, LEDs
la s1, COUNT
la s9, RUN

# Load addresses for relevant counter registers
la s3, counterStatus
la s4, counterControl
la s5, counterStartLow
la s6, counterStartHigh

# Load relevant button addresses
la s7, BUTTON
la s8, edgeCap

#/////////////////////////////////////////

#Set up the stack pointer
#Initialize the stack pointer
li sp, 0x10000 # Initialize sp to 0x10000

# Reset the LEDS
li t0, 0x0
sw t0, (s0)

# Reset EDGE bits of buttons
li t0, 0xF
sw t0, (s8)
	
jal CONFIG_TIMER # configure the Timer
jal CONFIG_KEYS # configure the KEYs port
	
/*Enable Interrupts in the NIOS V processor, and set up the address handling
location to be the interrupt_handler subroutine*/

# Turn on Interrupts in the mstatus register for NIOS V
li t0, 0x8 # Load bit 3 = 1 into t0
csrrs x0, mstatus, t0 # Load t0 into mstatus, enabling interrupts by making bit 3 (MIE) = 1

#Set the mtvec register to be the interrupt_handler location
la t0, interrupt_handler # Load the interrupt handler address into t0, which has label interrupt_handler in this code
csrrw x0, mtvec, t0 # Set the mtvec register to the address of interrupt_handler

#activate interrupts from IRQ18 (Pushbuttons) and IRQ16 (Timer)
li t0, 0x40000 # Load 1 into bit 18 of t0
li t1, 0x10000 # Load 1 into bit 16 of t1
add t0, t0, t1 # Add t1 to t0
csrrs x0, mie, t0 # Load t0 into mie, enabling interrupts for IRQ18 and IRQ16 by making bits 18 and 16 = 1

# One more step is needed to fully enable interrupts for the buttons
# This is done in CONFIG_TIMER

LOOP:
lw s2, 0(s1) # Get current count
sw s2, 0(s0) # Store count in LEDs
j LOOP


# This master interrupt handler will decide which sub-handler to call
interrupt_handler:
# Save to-be-used-register values as per good practice
# Also save the RA, since this interrupt handler calls subroutines
addi sp, sp, -12
sw t0, 0(sp)
sw t1, 4(sp)
sw ra, 8(sp)

# Decide if button or timer interrupted based on value of mcause

# Load value of mcause
csrr t0, mcause

# Load exact value corresponding to IRQ18 causing an interrupt in mcause
li t1, 0x80000012

# Below will jump if button caused the interrupt (IRQ18 = 1)
beq t0, t1, buttonIntHandle

# Otherewise, timer caused interrupt
call interruptTimer
j finishIntHandler

# Button caused interrupt (IRQ18 = 1)
buttonIntHandle:
call interruptKeys

finishIntHandler:
# Restore saved register values
lw t0, 0(sp)
lw t1, 4(sp)
lw ra, 8(sp)
addi sp, sp, 12

mret

# Timer caused interrupt (IRQ16 = 1)
# When this interrupt triggers, the count needs to be incremented or reset
interruptTimer:
# Save to-be-used-register values as per good practice
addi sp, sp, -8
sw t0, 0(sp)
sw t1, 4(sp)

# First reset the interrupt by writing 0 to the timer's Status register
# This also resets TO
li t0, 0x0
sw t0, (s3)

# Check if the count is at the max of 255
lw t0, (s1)
li t1, 255
bne t0, t1, justIncrement

# If COUNT = 255, reset it to 0
li t0, 0
sw t0, (s1)
j finishIntTimer

# If COUNT != 255, just increment it by RUN
justIncrement:
lw t0, (s1) # Load COUNT
lw t1, (s9) # Load RUN
add t0, t0, t1 # Add RUN (could be 1 or 0) to COUNT
sw t0, (s1)

finishIntTimer:
# Restore register values
lw t0, 0(sp)
lw t1, 4(sp)
addi sp, sp, 8

# Return
ret

interruptKeys:
# Save to-be-used-register values as per good practice
addi sp, sp, -12
sw t0, 0(sp)
sw t1, 4(sp)
sw t2, 8(sp)

# Check which button was pressed
# Load EDGE bits into t0
lw t0, (s8)

# Stop/Start if KEY0 pressed
li t1, 0x1
beq t0, t1, stopstartToggle

# Double counter if KEY1 pressed
li t1, 0x2
beq t0, t1, doubleSpeed

# Half counter if KEY2 pressed
li t1, 0x4
beq t0, t1, halfSpeed

# Do nothing if KEY3 pressed
j finishIntKeys


stopstartToggle:
# Load current value of RUN
lw t0, (s9)

# Check if RUN is zero or one
beqz t0, runZeroToOne
# If RUN = 1, change it to zero
runOneToZero:
li t1, 0x0
sw t1, (s9)
j finishIntKeys

# If RUN = 0, change it to one
runZeroToOne:
li t1, 0x1
sw t1, (s9)
j finishIntKeys


# Double timer speed, to a minimum of 12,500,000 counts
doubleSpeed:
# To check if timer speed is already at max (timer count limit is at its min)

lw t0, (s5) # Load lower 16 bits of current count limit
lw t1, (s6) # Load higher 16 bits of current count limit
# Load full 32-bit current count limit
slli t1, t1, 0x10
add t0, t0, t1

li t1, 0xBC20 # Load lower 16 bits of min count limit
li t2, 0xBE # Load higher 16 bits of min count limit

# Load full 32-bit min count value
slli t2, t2, 0x10
add t1, t1, t2

ble t0, t1, finishIntKeys # Compare current count with min count limit

# Counter speed can be increased
# Set the timer's control to stopped, continuous, interrupt mode -> ...1011 -> 0xB
li t0, 0x7
sw t0, (s4)

# Get current count limit
lw t0, (s5) # Get lower 16 bits
lw t1, (s6) # Get higher 16 bits

slli t1, t1, 0x10 # Shift higher 16 bits 16 bits left

add t0, t0, t1 # Get complete count limit by adding higher + lower bits

# Divide count limit by two
srli t0, t0, 1

# Extract lower 16 bits and store it

addi t1, t0, 0
andi t1, t1, 0xFF # Extract lower 8 bits

addi t2, t0, 0
srli t2, t2, 0x8 # Make upper 8 bits the lower 8 bits
andi t2, t2, 0xFF # Extract higher 8 bits of original
slli t2, t2, 0x8 # Move higher 8 bits back to original position

add t1, t1, t2 # Add the two 8-bit values to get the lower 16 bits

sw t1, (s5) # Store it

# Extract higher 16 bits and store it
addi t1, t0, 0
srli t1, t1, 0x10
sw t1, (s6)

# Start timer again in started, continuous, interrupt mode -> ...0111 -> 0x7
li t0, 0x7
sw t0, (s4)

# Wrap up
j finishIntKeys

# Half timer speed, to a maximum of 50,000,000 counts
halfSpeed:
# To check if timer speed is already at min (timer count limit is at its max)
lw t0, (s5) # Load lower 16 bits of current count limit
lw t1, (s6) # Load higher 16 bits of current count limit
# Load full 32-bit current count limit
slli t1, t1, 0x10
add t0, t0, t1

li t1, 0xF080 # Load lower 16 bits of max count limit
li t2, 0x2FA # Load higher 16 bits of max count limit
# Load full 32-bit max count value
slli t2, t2, 0x10
add t1, t1, t2

bge t0, t1, finishIntKeys # Compare current limit with max

# Counter speed can be decreased
# Set the timer's control to stopped, continuous, interrupt mode -> ...1011 -> 0xB
li t0, 0x7
sw t0, (s4)

# Get current count limit
lw t0, (s5) # Get lower 16 bits
lw t1, (s6) # Get higher 16 bits

slli t1, t1, 0x10 # Shift higher 16 bits 16 bits left

add t0, t0, t1 # Get complete count limit by adding higher + lower bits

# Multiply count limit by two
slli t0, t0, 1

# Extract lower 16 bits and store it

addi t1, t0, 0
andi t1, t1, 0xFF # Extract lower 8 bits

addi t2, t0, 0
srli t2, t2, 0x8 # Make upper 8 bits the lower 8 bits
andi t2, t2, 0xFF # Extract higher 8 bits of original
slli t2, t2, 0x8 # Move higher 8 bits back to original position

add t1, t1, t2 # Add the two 8-bit values to get the lower 16 bits

sw t1, (s5) # Store it

# Extract higher 16 bits and store it
addi t1, t0, 0
srli t1, t1, 0x10
sw t1, (s6)

# Start timer again in started, continuous, interrupt mode -> ...0111 -> 0x7
li t0, 0x7
sw t0, (s4)

# Wrap up
j finishIntKeys

#/////////////////////////////////////

finishIntKeys:
# Reset EDGE bits of buttons
li t0, 0xF
la t1, edgeCap
sw t0, (t1)

# Restore register values
lw t0, 0(sp)
lw t1, 4(sp)
lw t2, 8(sp)
addi sp, sp, 12

# Return
ret

CONFIG_TIMER: 

# Code completed by me here

# Load the counter stop and start values
# To count to 0.25 seconds, 0.25 sec / 10 ns = 25,000,000 counts of the hardware timer
# That's 0x17D7840 in hex, with 0x7840 filling the lower 16-bits and 17D filling the upper
li t0, 0x7840
sw t0, (s5)
li t0, 0x17D
sw t0, (s6)

# Set the timer's control to started, continuous, interrupt mode -> ...0111 -> 0x7
li t0, 0x7
sw t0, (s4)

ret

CONFIG_KEYS: 

# Code completed by me here

/*Allow interrupts on the pushbutton's interrupt mask register, and any 
#additional set up for the pushbuttons */
li t0, 0xF # Store 1111 (F in hex) at t0
sw t0, 8(s7) # Store 1111 at pushbutton's mask register - enabling interrupts for all 4 buttons

# VERY IMPORTANT: Clear EDGE register of pushbutton so previous presses don't carry over
li t0, 0xF
sw t0, (s8) # Write 1111 to the EDGE register of pushbuttons to reset the EDGE response for each of the 4 buttons

ret

.data
/* Global variables */

.align 4

.global  COUNT
COUNT:  .word    0x0            # used by timer

.global  RUN                    # used by pushbutton KEYs
RUN:    .word    0x1            # initial value to increment COUNT

.equ LEDs,  0xFF200000
.equ BUTTON, 0xFF200050
.equ edgeCap, 0xFF20005C
.equ TIMER, 0xFF202000

.equ counterStatus, 0xFF202000
.equ counterControl, 0xFF202004
.equ counterStartLow, 0xFF202008
.equ counterStartHigh, 0xFF20200C


.end
