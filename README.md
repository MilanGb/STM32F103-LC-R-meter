# STM32F103-LC-R-meter
Easy precision hoby meter

STM32F103C8 This is simply precision LC meter 0.0 nH to tens mH, 0.0 pF to hundrets nF and ohmmeter from 0.0 to 33MOhm with automatic scale. Display is LCD1602 A, coneected with 4 data D4,D5,D6,D7 are PB15,14,13,12; EN, RS are PA8,9; RW is permanently on ground, V0 for LCD connect to GND via 2.2(1.8) kOhm res to GND. Suply voltage LCD is 5V. Library is very simply. Very important is chain timers TIM1 and TIM4 started with TIM3 (prescaler to 1s) for measure resonans frequency LC oscillator. Oscillator is easy with OP amplifier LF353 with J-fet imputs, gain bandwitch - 4Mhz, power suply is +5V,GND,-5V. In my case is used L on ferit-core with inductans around 32 microHenry, after calibrations is saved to memory and cannot by bigger how 65 microHenry, size memory is 65535. Working capacitor is 1nF on 250V (styroflex). Feedback capacitors have 270 pF. Inductor and capacitor must have good quality for frequeny stability and precision measure. Calibrations capacitor must by very precision 1% or better, after calibrations we can compare measure value with this capacitors. 
  Calibration is in more steps. On first is need set working capacitor with feedback cap. + parasitic capacitans (1nF + 270pF + 10pf..). Next calculate 10x Inductors:
                1
  F = ------------------------                         L = --------------------
       
