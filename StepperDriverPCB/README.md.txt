## ⚠️ Hardware Advisory – Known Issue with Current PCB Revision

The current version of the PCB has a known hardware issue: the **BOOT0** pin is shared with the **CAN bus RX** line.  
When a CAN transceiver is connected, this causes the BOOT0 pin to be pulled **high**, which results in the microcontroller always starting in **programming mode** instead of executing the firmware.

This issue will be resolved in a future revision of the PCB.
