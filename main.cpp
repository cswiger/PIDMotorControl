/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
extern "C" {
#include <project.h>
}
#include <PID.h>
    
char *itoa(int val, int base) {
    static char buf[32] = {0};
    int i = 30;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    return &buf[i+1];
}

int main()
{
    
//    char buffer[10];  // for UART debugging if needed
    
    double Position;

    //PID controller constants
    double KP = 100; //position multiplier (gain)
    double KI = 3.; // Intergral multiplier (gain) - was 0.05 - use 0 for testing, simple P controller
    double KD = .2; // derivative multiplier (gain)

    // The Output variable to the motor driver
    double ms;
    int32 msi;
    // Setpoint we derive (map) from the pot
    double target;
//    int32 targeti;

    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    CyGlobalIntEnable; 
    
    PID myPID(&Position, &ms, &target,KP,KI,KD, DIRECT);
    myPID.SetOutputLimits(-32767,32767);
    myPID.SetSampleTime(1);
    myPID.SetMode(AUTOMATIC);
    
    QuadD_Start();
    QuadD_TriggerCommand(QuadD_MASK, QuadD_CMD_RELOAD);
    QuadDIn_Start();
    QuadDIn_TriggerCommand(QuadDIn_MASK, QuadDIn_CMD_RELOAD);
    
    UART_USB_Start(); 
    Millis_Start();
    PWM_Start();
    
    while(1)
    {
        
        target = (double)(8*(32768-QuadDIn_ReadCounter())+32768);

/*
        targeti = (uint32)target;
        strcpy(buffer,itoa(targeti,10));
        UART_USB_UartPutString("Target: ");
        UART_USB_UartPutString(buffer);
        UART_USB_UartPutString("\n\r");
 */
 
        Position = (double)QuadD_ReadCounter();
        
        myPID.Compute();
        
        if (ms >= 0) {
            INA_Write(0);
            INB_Write(1);
        } else {
            INA_Write(1);
            INB_Write(0);
            ms = -ms;
        }
        
        msi = int32(ms);
            
        PWM_WriteCompare(msi);
    }
}

/* [] END OF FILE */
