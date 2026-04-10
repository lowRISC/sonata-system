# Inter-integrated circuit (I2C) host

For the I2C block in Sonata, we use the OpenTitan IP.
The version of the IP that is vendored in has documentation that you can find [here](https://github.com/lowRISC/opentitan/tree/a78922f14a8cc20c7ee569f322a04626f2ac6127/hw/ip/i2c/doc).
Another useful reference is the [existing driver](https://github.com/microsoft/cheriot-rtos/blob/main/sdk/include/platform/sunburst/platform-i2c.hh) for this block in the CHERIoT RTOS repository.
We hardcode this block to be in host mode, so you can ignore the target functionality, including the following registers: `ovrd`, `val`, `target_id`, `acqdata` and `txdata`.
Other than those, you can find the [register definitions here](https://github.com/lowRISC/opentitan/blob/a78922f14a8cc20c7ee569f322a04626f2ac6127/hw/ip/i2c/doc/registers.md).
The registers `0x00` - `0x10` are also not accessible.
The control register is just hardwired to be in host mode.

For Sonata, we include two I2C blocks.
The registers of the second I2C block can be accessed with an additional `0x1000` offset.
These can be connected to any of these I2C targets:
- Two for the QWIIC connectors.
- One for the mikroBUS.
- One for the Raspberry Pi hat.
- One for the Arduino shield.
