# Motor Control Library Release Notes

Motor control library provides implementation of motor control algorithm to various control methods, control types and different motors. 


### What's Included?

Below table provides details of the different control type, controlled entity, feedback type and start up method supported by the motor control library.

**Control Methods**


 |Control Type       | Controlled Entity  |Feedback Type       |Start up Method          | 
 |:----------------- | :------------------|:----------------   |:------------------------| 
 |Open Loop          |Voltage             |N.A.                |N.A.                     | 
 |FOC in RFO         |Current             |Sensorless          |Rotor Pre-Alignment      | 
 |FOC in RFO         |Current             |Sensorless          |Six Pulse Injection      | 
 |FOC in RFO         |Current             |Sensorless          |High Frequency Injection | 
 |FOC in RFO         |Current             |Sensorless          |Dyno Mode                | 
 |FOC in RFO         |Current             |Encoder             |Rotor Pre-Alignment      | 
 |FOC in RFO         |Current             |Hall Sensor         |N.A.                     | 
 |TBC in BC          |Current             |Hall Sensor         |N.A.                     | 
 |TBC in TC          |Current             |Hall Sensor         |N.A.                     | 
 |FOC in SFO         |Torque              |Sensorless          |Rotor Pre-Alignment      | 
 |FOC in SFO         |Torque              |Sensorless          |Six Pulse Injection      | 
 |FOC in SFO         |Torque              |Sensorless          |High Frequency Injection | 
 |FOC in SFO         |Torque              |Sensorless          |Dyno Mode                | 
 |FOC in RFO or SFO  |Speed               |Sensorless          |Rotor Pre-Alignment      | 
 |FOC in RFO or SFO  |Speed               |Sensorless          |Six Pulse Injection      | 
 |FOC in RFO or SFO  |Speed               |Sensorless          |High Frequency Injection | 
 |FOC in RFO or SFO  |Speed               |Sensorless          |Open-Loop Volt/Hz        | 
 |FOC in RFO         |Speed               |Encoder             |Rotor Pre-Alignment      | 
 |FOC in RFO         |Speed               |Hall Sensor         |N.A.                     | 
 |TBC in BC          |Speed               |Hall Sensor         |N.A.                     | 
 |TBC in TC          |Speed               |Hall Sensor         |N.A.                     | 


### What Changed?

#### v1.8.0
* Initial release (Refer tot he above table for the supported features supported in the release)

### Supported Software and Tools
This version of the motor control library  was validated for compatibility with the following Software and Tools:

 | Software and Tools                        | Version                          | 
 | :---                                      | :-----------------------------   | 
 | ModusToolbox™ Software Environment        | 3.1.0                            | 
 | GCC Compiler                              | 11.3.1                           | 
 | IAR Compiler                              | 9.50.2                           | 
 | Motor Control Kit                         | KIT_XMC7200_DC_V1                | 
 | MCU                                       | XMC7000                          | 


Minimum required ModusToolbox™ Software Environment: v3.3

### More information
Use the following links for more information, as needed:
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2019-2023.
