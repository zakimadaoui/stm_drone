## ADC on-off control
The ADC can be powered-on by setting the ADON bit in the ADC_CR2 register. When the ADON bit is set for the first time, it wakes up the ADC from Power Down mode.

Conversion starts when ADON bit is set for a second time by software after ADC power-up time (tSTAB).

The conversion can be stopped, and the ADC put in power down mode by resetting the ADON bit. In this mode the ADC consumes almost no power (only a few μA).


## Channel selection
 The regular group is composed of up to 16 conversions. The regular channels and their order in the conversion sequence must be selected in the ADC_SQRx registers. The total number of conversions in the regular group must be written in the L[3:0] bits in the ADC_SQR1 register.



## Single conversion mode
In Single conversion mode the ADC does one conversion. This mode is started either by
setting the ADON bit in the ADC_CR2 register (for a regular channel only) or by external
trigger (for a regular or injected channel), while the `CONT` bit is 0.
Once the conversion of the selected channel is complete:

 If a regular channel was converted:
– The converted data is stored in the 16-bit ADC_DR register  
– The EOC (End Of Conversion) flag is set  in ADC_SR register
– and an interrupt is generated if the EOCIE is set.  

If an injected channel was converted:
– The converted data is stored in the 16-bit ADC_DRJ1 register  
– The JEOC (End Of Conversion Injected) flag is set  
– and an interrupt is generated if the JEOCIE bit is set.  

The ADC is then stopped.  

`Software clears the EOC bit`


## Scan mode

This mode is used to scan a group of analog channels. Scan mode can be selected by setting the SCAN bit in the ADC_CR1 register. Once this bit is set, ADC scans all the channels selected in the ADC_SQRx registers (for regular channels) or in the ADC_JSQR (for injected channels). A single conversion is performed for each channel of the group. After each end of conversion the next channel of the group is converted automatically. If the CONT bit is set, conversion does not stop at the last selected group channel but continues again from the first selected group channel. 

When using scan mode, DMA bit must be set and the direct memory access controller is used to transfer the converted data of regular group channels to SRAM after each update of the ADC_DR register.

## Calibration
The ADC has a built-in self calibration mode. Calibration significantly reduces accuracy errors due to internal capacitor bank variations. During calibration, an error-correction code (digital word) is calculated for each capacitor, and during all subsequent conversions, the error contribution of each capacitor is removed using this code. 
  
  
Calibration is started by setting the CAL bit in the ADC_CR2 register. Once calibration is over, the CAL bit is reset by hardware and normal conversion can be performed. It is recommended to calibrate the ADC once at power-on. The calibration codes are stored in the ADC_DR as soon as the calibration phase ends. It is recommended to perform a calibration after each power-up. Before starting a calibration, the ADC must have been in power-on state (ADON bit = ‘1’) for at least two ADC clock cycles.

## Data alignment

ALIGN bit in the ADC_CR2 register selects the alignment of data stored after conversion. use the `RIGHT alignment mode`


## Channel-by-channel programmable sample time
ADC samples the input voltage for a number of ADC_CLK cycles which can be modified using the SMP[2:0] bits in the ADC_SMPR1 and ADC_SMPR2 registers. Each channel can be sampled with a different sample time. The total conversion time is calculated as follows: 
 
Tconv = Sampling time + 12.5 cycles
Example:
With an ADCCLK = 14 MHz and a sampling time of 1.5 cycles:
Tconv = 1.5 + 12.5 = 14 cycles = 1 μs



## DMA requests

Since converted regular channels value are stored in a unique data register, it is necessary to use DMA for conversion of more than one regular channel. This avoids the loss of data already stored in the ADC_DR register. Only the end of conversion of a regular channel generates a DMA request, which allows the transfer of its converted data from the ADC_DR register to the destination location selected by the user. Only ADC1 and ADC3 have this DMA capability. ADC2-converted data can be transferred in dual ADC mode using DMA thanks to master ADC1.





# Registers

## ADCx_CR1 bits

SCAN: Scan mode
This bit is set and cleared by software to enable/disable Scan mode. In Scan mode, the inputs selected through the ADC_SQRx or ADC_JSQRx registers are converted.
`0: Scan mode disabled`  
1: Scan mode enabled

## ADCx_CR2 bits


`ADON`: A/D converter ON / OFF
This bit is set and cleared by software. If this bit holds a value of zero and a 1 is written to it then it wakes up the ADC from Power Down state. Conversion starts when this bit holds a value of 1 and a 1 is written to it. The application should allow a delay of tSTAB between power up and start of conversion. Refer to Figure 23.

0: Disable ADC conversion/calibration and go to power down mode.  
1: Enable ADC and to start conversion  


`CONT`: Continuous conversion
This bit is set and cleared by software. If set conversion takes place continuously till this bit is reset.

0: Single conversion mode  
1: Continuous conversion mode  



`DMA`: Direct memory access mode
This bit is set and cleared by software. Refer to the DMA controller chapter for more details.
0: DMA mode disabled
1: DMA mode enabled
Only ADC1 and ADC3 can generate a DMA request.


`ALIGN`: Data alignment
This bit is set and cleared by software. Refer to Figure 27.and Figure 28.

0: Right Alignment  
1: Left Alignment  


`RSTCAL`: Reset calibration
This bit is set by software and cleared by hardware. It is cleared after the calibration registers are initialized.
0: Calibration register initialized. 
1: Initialize calibration register. 


`CAL`: A/D Calibration: This bit is set by software to start the calibration. It is reset by hardware after calibration is complete.
0: Calibration completed  
1: Enable calibration  


`EXTSEL[2:0]`: External event select for regular group These bits select the external event used to trigger the start of conversion of a regular group:

For ADC1 and ADC2, the assigned triggers are:  
000: Timer 1 CC1 event  
001: Timer 1 CC2 event  
010: Timer 1 CC3 event  
011: Timer 2 CC2 event  
100: Timer 3 TRGO event  
101: Timer 4 CC4 event  
110: EXTI line 11/TIM8_TRGO event (TIM8_TRGO is available only in high-density and XL-density devices)  
111: `SWSTART`  


`SWSTART` bit Start conversion of regular channels This bit is set by software to start conversion and cleared by hardware as soon as conversion starts. It starts a conversion of a group of regular channels if SWSTART is
selected as trigger event by the EXTSEL[2:0] bits.
0: Reset state
1: Starts conversion of regular channels

## ADC sample time register 2 (ADC_SMPR2)

`SMPx[2:0]`: Channel x (`interested in channel 9`) Sample time selection These bits are written by software to select the sample time individually for each channel. During sample cycles channel selection bits must remain unchanged. 

000: 1.5 cycles  
001: 7.5 cycles  
010: 13.5 cycles  
011: 28.5 cycles  
100: 41.5 cycles  
101: 55.5 cycles  
110: 71.5 cycles  
111: 239.5 cycles  


## ADC regular sequence register 1 (ADC_SQR3)

there are 16 regular ADC channels, the L[3:0] allows you to define the nbr of the sequence of channels (for ex: 3 channels to check in the sequence).



`L[3:0]`: Regular channel sequence length
These bits are written by software to define the total number of conversions in the regular channel conversion sequence.

`0000: 1 conversion`   
0001: 2 conversions  
.....  
1111: 16 conversions  


`SQx[4:0]`: these bits hold which channel is the xth conversion in regular sequence
These bits are written by software with the channel number (0..17) assigned as the 16th in
the conversion sequence. 

From 0 to 16 requires 5-bits (there are 0->17 in some devices)!

The temperature sensor is connected to channel ADCx_IN16 and the internal reference voltage V REFINT is connected to ADCx_IN17. These two internal channels can be selected and converted as injected or regular channels. The sensor and VREFINT are only available on the master ADC1 peripheral.


So for me, I need a single conversion in CH9 so I put "0000" in L[3:0] of ADC_SQR1 and "01001" in SQ1[4:0] of ADC_SQR3 


##  ADC regular data register (ADC_DR)

`DATA[15:0]`: Regular data
These bits are read only. They contain the conversion result from the regular channels. The data is left or right-aligned as shown in Figure 27 and Figure 28.