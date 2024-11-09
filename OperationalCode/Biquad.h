/*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#pragma once

typedef struct
{
    // H(s)=k*((s/wz1+1)(s/wz2+1))/((s/wp1+1)(s/wp2+1))
    // H(s)=(a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
    // H(z)=(m0+m1*z^-1+m2*z^-2)/(n0+n1*z^-1+n2*z^-2)
    // a0=k,	a1=k*(1/wz1+1/wz2),	a2=k/(wz1*wz2)
    // b0=1,	b1=1/wp1+1/wp2,		b2=1/(wp1*wp2)
    // m0=a0+a1*fs+a2*fs^2,	m1=-(a1*fs+2*a2*fs^2),	m2=a2*fs^2
    // n0=b0+b1*fs+b2*fs^2,	n1=-(b1*fs+2*b2*fs^2),	n2=b2*fs^2
    // n0 is always normalized to be 1.0f
    float fs;
    float k;
    float wz[2];
    float wp[2];
    float a[3];
    float b[3];
    float m[3];
    float n[3];
    float d[3];
    float input;
    float output;
} BIQUAD_t;

// Pole/Zero, Continuous, and Discrete initializations are supported
void BIQUAD_PoleZeroInit(BIQUAD_t* biquad, const float fs, const float k, const float wz[2], const float wp[2]);
void BIQUAD_ContinuousInit(BIQUAD_t* biquad, const float fs, const float a[3], const float b[3]);
void BIQUAD_DiscreteInit(BIQUAD_t* biquad, const float fs, const float m[3], const float n[3]);

void BIQUAD_Reset(BIQUAD_t* biquad, const float output);

float BIQUAD_Run(BIQUAD_t* biquad, float input);