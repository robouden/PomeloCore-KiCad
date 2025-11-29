# PomeloCore Firmware Software Flow Summary

### **1. Initialization Flow**
The system starts by configuring hardware and restoring state.

*   **System & Power**: Initialize system clocks, voltage regulators, and NVM (Flash) settings.
*   **GPIO Configuration**: Set up pins for LED, USB VBUS, Analog Front End (AFE), Peak Detector, Ramp Generator, and Triggers.
*   **Peripherals Setup**:
    *   **RTC & Timers**: For timekeeping and gamma pulse timing.
    *   **EIC**: External Interrupts for triggers.
    *   **ADC & DAC**: For sampling signals and controlling High Voltage (HV).
    *   **I2C**: For external sensors (Temperature).
    *   **UART & USB**: For communication.
*   **State Restoration**: Load parameters from NVM (Flash) and apply them (HV settings, coincidence mode, etc.).

### **2. Main Loop Flow**
The infinite loop continuously handles communication, data processing, and power management.

```mermaid
graph TD
    Start(Start Loop) --> USB_Check{USB Connected?}
    USB_Check -- Yes --> USB_Handler[Process USB Data]
    USB_Check -- No --> UART_Handler
    USB_Handler --> UART_Handler[Process UART Data]
    
    UART_Handler --> Data_Out{Data Output Pending?}
    Data_Out -- Yes --> Process_FIFO[Process Gamma FIFO]
    Process_FIFO --> Send_Data[Send Pulse/Energy Data to USB/UART]
    Data_Out -- No --> VBUS_Check
    Send_Data --> VBUS_Check
    
    VBUS_Check{VBUS Changed?}
    VBUS_Check -- Plugged --> Enable_USB[Enable USB & High Perf Clock]
    VBUS_Check -- Unplugged --> Disable_USB[Disable USB & Low Power Clock]
    VBUS_Check -- No Change --> Temp_Comp
    
    Enable_USB --> Temp_Comp
    Disable_USB --> Temp_Comp
    
    Temp_Comp{Temp Comp Run?}
    Temp_Comp -- Yes --> Update_HV[Update HV Bias]
    Temp_Comp -- No --> Sync_Trig
    Update_HV --> Sync_Trig
    
    Sync_Trig{Sync Trigger?}
    Sync_Trig -- Yes --> Gen_Pulse[Generate Sync Pulse]
    Sync_Trig -- No --> Sleep_Check
    Gen_Pulse --> Sleep_Check
    
    Sleep_Check{Can Sleep & No USB?}
    Sleep_Check -- Yes --> Sleep[Enter Standby Mode]
    Sleep_Check -- No --> Start
    Sleep --> Start
```

### **3. Command Processing Flow**
Incoming characters from USB or UART trigger the `command_data_handler`.

*   **`g`**: Get CPM (Counts Per Minute).
*   **`u`**: Get uSv/h (Dose rate).
*   **`m`**: Measure (Returns JSON with CPM, uSv/h, Temperature).
*   **`i`**: Get SiPM current and diagnostics.
*   **`r`**: Reload parameters from Flash.
*   **`p`**: Enter Parameter Input Mode (Accepts configuration string).
*   **`/`**: Enable HV Boost.
*   **`*`**: Disable HV Boost.

### **4. Key Interrupt/Background Tasks**
*   **Trigger Callback**: Detects radiation pulses, captures peak energy, and stores in `gammaFifo`.
*   **Timer/RTC Callbacks**: Manage measurement windows and periodic tasks.
*   **HV Load Callback**: Monitors the current draw of the High Voltage supply.
