# MS51_ADC_PWM_LowPassFilter
 MS51_ADC_PWM_LowPassFilter

1. Initial ADC channel x 3 (P0.4 , ADC_CH5 , , P0.3 , ADC_CH6 , P1.1 , ADC_CH7)

2. Initial PWM x 1 (P1.5 , PWM0_CH5) , with freq : 16K

3. by change ADC channel(P0.4 , ADC_CH5) , to change PWM output duty , with duty which able to set 4 digit

ex : duty 83.5% , parameter d of PWM0_CHx_Init set 8350 

ex : duty 26% , parameter d of PWM0_CHx_Init set 2600

4. ADC sampling with 4 define , to get ADC value 

- ENABLE_ADC_RAW_DATA

- ENABLE_ADC_GET_N_DEL_X

- ENABLE_ADC_AVG

- ENABLE_ADC_LOW_PASS_FILTER


5. enable define : _debug_log_xxx to check different function log

below is screen capture , with terminal log message , and PWM channel scope measurement

![image](https://github.com/released/MS51_ADC_PWM_LowPassFilter/blob/main/capture.jpg)

