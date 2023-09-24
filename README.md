# Lab 5: FreeRTOS

### 3. Assignment

#### 3.1.

Write a program using FreeRTOS that blinks the LEDs at different intervals, LED1
= 1 second, LED2 = 2 second, LED3 = 3 second, LED4 = 4 second. Pressing a button
on the board (1-2) should make the corresponding LED (1-2) stay lit for 10
seconds while the others keep blinking. If both buttons are pressed, one after
another, both LEDs should light up. When 10 seconds has passed, the affected LED
should go back into its original state, i.e. pressing the button should **NOT**
make the LEDs blink out of sync. Make the various task output their actions to
the serial port.

#### 3.2.

Firstly, you will implement three different tasks that have periods, release
times and priorities in such a manner that the priority inversion (see RTOS
lecture slides) takes place. If done properly, the execution of a correctly
implemented priority inversion will result in a deadline miss of the tasks. To
visualize the execution of the different tasks, you are required to output
certain keypoints to the UART listed below:

- When a task is released for execution – “Task x started”
- When a task finishes its execution – “Task x finished”
- When a task successfully takes a semaphore – “Task x sem take”
- When a task releases a semaphore – “Task x sem give”
- When a task starts its workload – “Task x started its workload”

#### 3.3.

Write a software function to detect the deadline miss when the task set in (part
2 the assignment) is executed. You should write the following information via
the UART: which task misses its deadline and by how much time the deadline is
missed. All tasks should have this deadlines miss detection.

##### Note 1:

A good idea is to discuss the design of the priority inversion with the Lab
assistant and get approval before starting to code. It is not mandatory to
discuss, but encouraged, as it is quite easy to misinterpret priority inversion.
The design must also be attached with the report.

##### Note 2:

VtaskDelay and VtaskDelayUntil are not an acceptable workloads for priority
inversion since they enforce a context switch between tasks.

##### Important Tip:

Read the study material provided in the corresponding RTOS lecture before
starting the design, there is an simple example of priority inversion that you
can follow as well.

### 4 Report

The report should include the following.

1. What is the main difference between preemptive and cooperative scheduling?
2. What is the difference between vTaskDelay() and vTaskDelayUntil()?
3. Design of Assignment 2 (priority inversion).

### 5 Optional Assignments

Measure the context switching time using a timer. How does it vary with the
number of active tasks? Plot context switch time as a function of number of
active tasks in a graph.
