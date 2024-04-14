# XIAOMI CyberGear Async API

Python Async API For Xiaomi CyberGear Micro Motor

Inspired by other sync python lib for CyberGear, the async api allows users to control motor in async framework, enabling real-time control and swift response.

#### The main features are:

- Efficient: Support Async Framework
- Flexible: Separate CAN Command Sending And Receiving Processes
- Friendly: Simple To Use, Easy To Modify

### Installation

Don't worry, the CyberGear Async API is easy to install

1. Install python if you don't have yet
    - Anaconda is recommended to be used. [Here is how to install it.](https://docs.anaconda.com/anaconda/install/)
    - Once you have Anaconda, you can run
    ```sh
    conda create -n cyberController python=3.12.2
    ```
    - After installation, you should activate it,
    ```sh
    conda avtivate cyberController
    ```
2. Clone this repo or download the zip file
3. Enter the folder
    ```sh
    cd CyberGearAsyncAPI
    ```
4. Install the dependencies for CyberGearAsyncAPI
    ```sh
    pip install -r requirements.txt
    ```

### Usage
1. Enable async environment and import CyberGear Async API

    ```python
    import asyncio
    import aioserial
    from cyberGearAsyncAPI import CANMotorAsyncController
    ```

2. Initial aioserial and CANMotorAsyncController instance
    ```python
    serial_port: str = "Your Serial Port"
    motor_id: int = 1 # Your Motor CAN ID
    main_can_id:int = 253 # Your CAN Connector ID
    aioserial_instance: aioserial.AioSerial = aioserial.AioSerial(port=serial_port, baudrate=921600)

    cyberGear = CANMotorAsyncController(aioserial_instance, motor_id=motor_id, main_can_id=main_can_id)
    ```

3. Set zero point for motor
    ```python
    await cyberGear.async_set_0_pos()
    ```

4. Enjoy CyberGear in debugging way
    ```python
    await cyberGear.async_enable() # Enable motor
    await cyberGear.async_set_mode(mode=3) # Set motor run mode

    await cyberGear.async_write_property(index=0x7006, value=I_q, data_type='f') # Set target q axis target current

    await asyncio.sleep(1) # Run the above command for 1 second
    angle, speed, torque = cyberGear.Motor_Status # Retrieve motor current status (angle, speed and torque)

    await cyberGear.async_disable() # Disable motor

    ```

5. Production code workflow instance
    - Current Mode
        ```python
        await cyberGear.async_enable() # Enable motor
        await cyberGear.async_set_mode(mode=3) # Set motor run mode

        while loop_condition: # Set run loop condition
            # Set break loop condition, such like the speed is over maximum value.
            if break_condition: 
                await cyberGear.async_disable() # Disable motor
                break
            # Set target q-axis current for motor, the index value 0x7006 can be found in manual
            await cyberGear.async_write_property(index=0x7006, value=I_q, data_type='f')  

            await asyncio.sleep(0.002) # Meet your control frequency
            
            angle, speed, torque = cyberGear.Motor_Status # Retrieve motor current status (angle, speed and torque)

            # You can set other interrupt method
            except KeyboardInterrupt: 
                await cyberGear.async_disable()
            
            finally:
                await cyberGear.async_disable() 
        ```
    
    - Speed Mode
        ```python
        await cyberGear.async_enable() # Enable motor
        await cyberGear.async_set_mode(mode=2) # Set motor run mode

        while loop_condition: # Set run loop condition
            # Set break loop condition, such like the speed is over maximum value.
            if break_condition: 
                await cyberGear.async_disable() # Disable motor
                break
            # Set target speed for motor, the index value 0x700A can be found in manual
            await cyberGear.async_write_property(index=0x700A, value=S_t, data_type='f')  

            await asyncio.sleep(0.002) # Meet your control frequency

            angle, speed, torque = cyberGear.Motor_Status # Retrieve motor current status (angle, speed and torque)

            # You can set other interrupt method
            except KeyboardInterrupt: 
                await cyberGear.async_disable()
            
            finally:
                await cyberGear.async_disable() 
        ```

    - Position Mode
        ```python
        await cyberGear.async_enable() # Enable motor
        await cyberGear.async_set_mode(mode=1) # Set motor run mode

        while loop_condition: # Set run loop condition
            # Set break loop condition, such like the speed is over maximum value.
            if break_condition: 
                await cyberGear.async_disable() # Disable motor
                break
            # Set target position for motor, the index value 0x7016 can be found in manual
            await cyberGear.async_write_property(index=0x7016, value=P_t, data_type='f')  

            await asyncio.sleep(0.002) # Meet your control frequency

            angle, speed, torque = cyberGear.Motor_Status # Retrieve motor current status (angle, speed and torque)
            
            # You can set other interrupt method
            except KeyboardInterrupt: 
                await cyberGear.async_disable()
            
            finally:
                await cyberGear.async_disable() 
        ```


