# EMBSYS-310-A-Final-Project
## Welcome to the final project

## Requirements

The B-L475E-IOT01A1 Evaluation Kit contains the following sensors:

* HTS221 Capacitive digital sensor for relative humidity and temperature
* LIS3MDL 3-axis Magnetometer
* LPS22HB MEMS nano pressure sensor
* LSM6DSL 3D accelerometer and 3D gyroscope
* VL53L0X Time-of-Flight ranging and gesture detection sensor
* M24SR64-Y Dynamic NFC/RFID tag IC
* MP34DT01 MEMS Microphone (x2)

### The requirements are:

1. Get at least one sensor initialized, configured and returning data
	1. Extra credit for doing this in an interrupt driven fashion
2. Process the data to make it human readable
3. Print the data to the console at a human-readable speed
	1. e.g., Less than 2 data points per second
	2. Or only print when things change enough to warrant it – and be able to force the change to demonstrate it!
4. Extra credit
	1. Change modes using push buttons
	2. Indicate things using the two LEDs
	
## Using the sensors

With the exception of the microphones, all sensors are connected to the microcontroller via the second I2C port.

I2C allows multiple devices on a single bus by giving each device a unique address. The addresses of each device are found in Table 3 in the UM2153 user manual for the development kit.

Most I2C devices contain a number of control and status registers. The data sheets for each sensor will give the register list. Register addresses are usually 8 or 16 bits wide. Registers are usually 8 bits wide. Larger values will occupy adjacent 8-bit registers.

You can investigate the I2C protocol and determine how to access these registers byte-by-byte, but the HAL contains functions that take the device and register address to read or write to.

Microphones are connected via the DFSDM interface, not the I2C. The HAL contains functions to access the DFSDM interface and retrieve values.

## Interrupts and Polling
### Polling

It is advised to get the system working with polling first, and implementing interrupts as an extra-credit option. In a polling system you request data, wait for the response, process it, then loop to the beginning and request data again.

The `HAL_Delay()` function can be used to pause execution for a given number of milliseconds e.g., `HAL_Delay(1000);` will cause the program to pause for 1 second.

### Interrupts

By week 6 we haven't spoken much about interrupt systems. Put simply, when an event occurs in the hardware that needs to be dealt with immediately, an interrupt is triggered. The processor stops what it is doing, executes code to handle the interrupt, then goes back to what it was doing.

The HAL makes it easy to create interrupt handlers for your code should you decide to. Interrupt handlers in the HAL execute 'callback functions' that use the `__weak` keyword to allow them to be overridden by a user-defined version.

e.g. if you want to execute code when the I2C finishes reading, the HAL defines the following callback:

```
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterRxCpltCallback could be implemented in the user file
   */
}
```

You can copy this function into your code, and delete the `__weak` prefix and the contents of the function, and write your own code.

NOTE: Interrupts for most devices are not enabled in STM32CubeMX. Open `embsys310_final_project.ioc` and for a given device in the left-hand menu go to the 'NVIC Settings' section and select the interrupt you wish to enable.

### Button Interrupt
In the default `embsys310_final_project.ioc` file in GitHub classroom, the user button is implemented as an External Interrupt rather than the GPIOs we have been studying. This means when the button is pressed the interrupt handler is called almost immediately.

The interrupt handler is the `EXTI15_10_IRQHandler()` function in `stm32l4xx_it.c`:
1. `EXTI15_10_IRQHandler()` calls `HAL_GPIO_EXTI_IRQHandler()` with the GPIO_Pin that triggered it
2. `HAL_GPIO_EXTI_IRQHandler()` calls `HAL_GPIO_EXTI_Callback()` with the GPIO_Pin that triggered it
3. `HAL_GPIO_EXTI_Callback()` is a `__weak` function that can be overridden in your own code

You can look into Atomic variables for passing data safely from an Interrupt Service Routine (ISR) to the main code should you wish to do it safely. For our application a simple global `bool` value set `true` when the button is pressed, and read and set `false` from the main loop should suffice. 
 
Alternately you can go back to STM32CubeMX and convert the pin to a standard GPIO Input and poll it.

# Important: Using STM32CubeMX to modify code
You can open the `embsys310_final_project.ioc` file in STM32CubeMX at any time to alter how the hardware is defined, e.g. to enable or disable interrupts, or to change LEDs to PWM. STM32CubeMX will go into the project files and modify the code appropriately.

Since STM32CubeMX will modify source code it is **vital** you only modify code between the USER CODE comments:

```
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
```


Code written in between the comments is safe from deletion by STM32CubeMX.

# Deliverables
There is a file located at `Core/Src/Final_Project_Readme.md` that should be filled in with instructions for the grader on how to use the project.

Provide the URL of your GitHub pull request in Canvas.

# GitHub Classroom Flow
## Cloning and working on project
1. Click on the GitHub Classroom link. This will create your own GitHub repo with starter code for the homework
2. Clone the repo locally: `git clone <repo>`
3. Create a working branch: `git checkout -b final_project`
4. Make incremental changes to the code, commit and push. I suggest doing this any time you would like to save the state of the code that you may want to go back to. The number of commits and their content is irrelevant for homework grading (have as many as you would like)
 
```
git add -A :/
git commit -m "A good description of the changes"
git push
<repeat as many times as you'd like>
```

## Create Pull Request
1. Open `<repo>` in a browser. You may already see a "Create new pull request" from your branch if you made recent changes. If not, select "Pull requests" and click "New". Leave `base: main` and modify `compare:` to point to your branch name. Create the new PR
2. If you find that you need to make more changes, just commit and push new changes to your branch - the PR will be updated

# Submission
1. Commit your working code to your Git repository and push to GitHub
2. In GitHub go to the green "<> Code" button, click on the down arrow and select "Download Zip"
    1. Name the zip file `embsys310_final_project_<lastname>_<firstname>.zip`
3. Create a GitHub pull request in your repository
4. In Canvas, submit the zip file as your assignment submission, and post a comment with the URL of your pull request to the assignment. Please only post the URL, not any other text in that comment as it makes it easier to open-as-link and get to the PR for the grader. Feel free to add other comments after the PR comment

## Grading
1. DO NOT close or merge the PR - we will do it for you as we're grading
2. To complete your homework, submit the PR link and zip file within Canvas (this step is mandatory - without something in Canvas, I won't be able to grade)
3. If I have any comments, I will leave them in the PR on GitHub. My comments in Canvas tend to be just pointing out the GitHub comments

## Troubleshooting:
- The first time you click a link for this classroom, you may need to associate your GitHub account to the classroom
- Some students reported they don't have access: this is a GitHub Classroom bug. If this happens, contact us immediately supplying the repo link as well as your GitHub account name. Create the repos as soon as possible to give us time to address any permissions issue before the assignments are due