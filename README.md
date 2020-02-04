# RZ_A2M_DRP_samples
This is a collection of sample programs that work on RZ/A2M boards.  
You can try Mbed OS for RZ/A2M with the following board.
- GR-MANGO beta version  
- [RZ/A2M Evaluation Board Kit](https://www.renesas.com/jp/en/products/software-tools/boards-and-kits/eval-kits/rz-a2m-evaluation-board-kit.html)  
- [SBEV-RZ/A2M](http://www.shimafuji.co.jp/products/1486)  
- [SEMB1402](http://www.shimafuji.co.jp/products/1505)  


## Overview
This is a collection of sample programs using DRP(Dynamically Reconfigurable Processor). DRP is the programmable hardware which have both the flexibility of software and the speed of hardware. The firmware which define processing, can be renewed immediately.  

Please see ``mbed-gr-libs/drp-for-mbed/TARGET_RZ_A2XX/r_drp/doc`` for details.  

The DRP program switches every 10 seconds. You can switch to the next program immediately by pressing ``USER_BUTTON0``.  


## About custom boot loaders
**<font color="Red">Attention! When using GR-MANGO, please use DAPLink without using a custom boot loader.</font>**  
This sample uses a custom boot loader, and you can drag & drop the "xxxx_application.bin" file to write the program.  

1. Hold down ``SW3 (UB0)`` and press the reset button. (Or turn on the power.)  
2. Connect the USB cable to the PC, you can find the ``MBED`` directory.  
3. Drag & drop ``xxxx_application.bin`` to the ``MBED`` directory.  
4. When writing is completed, press the reset button.  

**Attention!**  
This sample program uses custom boot loaders ``revision 4`` .  
For the first time only, you need to write a custom bootloader as following.  
[How to write a custom boot loader](https://github.com/d-kato/bootloader_d_n_d)  


## Development environment
Please refer to the following.  
https://github.com/d-kato/RZ_A2M_Mbed_samples
