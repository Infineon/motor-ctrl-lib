# Motor Control Library


### Overview

Motor Control Library represents integration of the implemented cross platform motor control algorithms with the MCU-family oriented Motor Control Driver Interface (MCDI).

<!--  **Table 1. Glossary** -->
<!--  | Acronym |            Expansion             | -->
<!--  |:-------:|:---------------------------------| -->
<!--  |   FOC   | Field Oriented Control           | -->
<!--  |   RFO   | Rotor Frame Orientation          | -->
<!--  |   SFO   | Stator Frame Orientation         | -->
<!--  |   TBC   | Trapezoidal or Block Commutation | -->
<!--  |   BC    | Block Commutation                | -->
<!--  |   TC    | Trapezoidal Commutation          | -->

**Table 1. Glossary**

<table style="border-collapse: collapse;">
  <tr>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Acronym</th>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Expansion</th>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC</td>
    <td style="border: 1px solid black; padding: 8px;">Field Oriented Control</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Rotor Frame Orientation</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Stator Frame Orientation</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TBC</td>
    <td style="border: 1px solid black; padding: 8px;">Trapezoidal or Block Commutation</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">BC</td>
    <td style="border: 1px solid black; padding: 8px;">Block Commutation</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TC</td>
    <td style="border: 1px solid black; padding: 8px;">Trapezoidal Commutation</td>
  </tr>
</table>

**The Motor Control Library algorithms set supports the following features:**

- ***Advanced Features***
    - Supporting <span style="color:bk">Rotor Frame Oriented FOC</span>
    - Supporting <span style="color:bk">Stator Frame Oriented FOC</span>
    - Supporting <span style="color:bk">Trapezoidal or Block Commutation</span> control
- ***Floating Point Base***
- ***GUI Integration*** : Easy monitoring and configuration of the various motor control parameters can be done via the GUI
    - <span style="color:bk">Configurator</span> view
    - <span style="color:bk">Test Bench</span> view
    - <span style="color:bk">Tools</span> such as <span style="color:bk">Oscilloscope</span>, <span style="color:bk">Motor Profiler</span>, etc.

**The Motor Control Driver Interface features include  :**

- ***Advanced Features***
    - Supporting <span style="color:bk">PSOC&trade; Control C3 </span> microcontrollers family
- ***Device Configurator Integration***


### Details

- Motor control library supports 25 different permutations of the control type, control entity, feedback type, and startup method as shown in the table above.
- In addition, both three-shunt and single-shunt configurations are supported, which results in more flexibility in supporting various applications.
- Note that the user will have flexibility to include or bypass the current loop when using *TBC in BC* mode.
- Bypassing the current loop can address low-cost BLDC applications with no shunts or ADCs.

<!--**Figure 1. Control Methods** -->
<!--  |    Control Type   | Controlled Entity | Feedback Type |      Startup Method      | -->
<!--  |:-----------------:|:-----------------:|:-------------:|:------------------------:| -->
<!--  |     Open-loop     |      Voltage      |      N.A.     |           N.A.           | -->
<!--  |     FOC in RFO    |      Current      |   Sensorless  |   Rotor Pre-Alignment    | -->
<!--  |     FOC in RFO    |      Current      |   Sensorless  |   Six Pulse Injection    | -->
<!--  |     FOC in RFO    |      Current      |   Sensorless  | High Frequency Injection | -->
<!--  |     FOC in RFO    |      Current      |   Sensorless  |         Dyno Mode        | -->
<!--  |     FOC in RFO    |      Current      |    Encoder    |   Rotor Pre-Alignment    | -->
<!--  |     FOC in RFO    |      Current      |  Hall Sensor  |           N.A            | -->
<!--  |     TBC in BC     |      Current      |  Hall Sensor  |           N.A            | -->
<!--  |     TBC in TC     |      Current      |  Hall Sensor  |           N.A            | -->
<!--  |     FOC in SFO    |      Torque       |   Sensorless  |   Rotor Pre-Alignment    | -->
<!--  |     FOC in SFO    |      Torque       |   Sensorless  |   Six Pulse Injection    | -->
<!--  |     FOC in SFO    |      Torque       |   Sensorless  | High Frequency Injection | -->
<!--  |     FOC in SFO    |      Torque       |   Sensorless  |         Dyno Mode        | -->
<!--  | FOC in RFO or SFO |      Speed        |   Sensorless  |   Rotor Pre-Alignment    | -->
<!--  | FOC in RFO or SFO |      Speed        |   Sensorless  |   Six Pulse Injection    | -->
<!--  | FOC in RFO or SFO |      Speed        |   Sensorless  | High Frequency Injection | -->
<!--  | FOC in RFO or SFO |      Speed        |   Sensorless  |     Open-Loop Volt/Hz    | -->
<!--  |    FOC in RFO     |      Speed        |    Encoder    |   Rotor Pre-Alignment    | -->
<!--  |    FOC in RFO     |      Speed        |  Hall Sensor  |           N.A            | -->
<!--  |    TBC in BC      |      Speed        |  Hall Sensor  |           N.A            | -->
<!--  |    TBC in TC      |      Speed        |  Hall Sensor  |           N.A            | -->

**Table 2. Control Methods**

<table style="border-collapse: collapse;">
  <tr>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Control Type</th>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Controlled Entity</th>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Feedback Type</th>
    <th style="border: 1px solid black; padding: 8px; background-color: #f0f0f0;">Startup Method</th>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">Open-loop</td>
    <td style="border: 1px solid black; padding: 8px;">Voltage</td>
    <td style="border: 1px solid black; padding: 8px;">N.A.</td>
    <td style="border: 1px solid black; padding: 8px;">N.A.</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Rotor Pre-Alignment</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Six Pulse Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">High Frequency Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Dyno Mode</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Encoder</td>
    <td style="border: 1px solid black; padding: 8px;">Rotor Pre-Alignment</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TBC in BC</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TBC in TC</td>
    <td style="border: 1px solid black; padding: 8px;">Current</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Torque</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Rotor Pre-Alignment</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Torque</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Six Pulse Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Torque</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">High Frequency Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Torque</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Dyno Mode</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO or SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Rotor Pre-Alignment</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO or SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Six Pulse Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO or SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">High Frequency Injection</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO or SFO</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Sensorless</td>
    <td style="border: 1px solid black; padding: 8px;">Open-Loop Volt/Hz</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">FOC in RFO</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TBC in BC</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
  <tr>
    <td style="border: 1px solid black; padding: 8px;">TBC in TC</td>
    <td style="border: 1px solid black; padding: 8px;">Speed</td>
    <td style="border: 1px solid black; padding: 8px;">Hall Sensor</td>
    <td style="border: 1px solid black; padding: 8px;">N.A</td>
  </tr>
</table>

Three major control types are selectable as <span style="color:bk">build configurations</span>  (via Make variables CTRL_METHOD_RFO, CTRL_METHOD_SFO & CTRL_METHOD_TBC ).


### Quick Start with the MCDI

Refer to the [API Reference Guide Quick Start Guide](https://infineon.github.io/motor-ctrl-lib/html/group__group__mcdi__general.html) section for step-by-step instructions on enabling the ModusToolbox™ Motor Control Driver Interface.


### More information

Detailed information on supported toolchains (make variable 'TOOLCHAIN'), kits (make variable 'TARGET'), Hardware setup , Software setup, creating project in ModusToolbox with all the other assets for a particular kit and code example etc. are available in the README file of MOTOR_DEMO Code Example.

**Resources  | Links**
- Supported motor control kits | KIT_XMC7200_DC_V1  and KIT_PSC3M5_CC2 
- Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1). Peripheral Driver Library (PDL) [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1). Hardware Abstraction Layer (HAL) library [retarget-io](https://github.com/Infineon/retarget-io). Utility library to retarget STDIO messages to a UART port
- Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox). ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSoC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.
- [ModusToolbox Power Conversion API Reference Guide](https://infineon.github.io/motor-ctrl-lib/html/group__group__mcdi__general.html)
- [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
- [Infineon Technologies AG](https://www.infineon.com)

<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2025. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.

