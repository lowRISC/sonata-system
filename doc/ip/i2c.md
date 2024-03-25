# Inter-integrated circuit (I2C) host

For the I2C block in Sonata, we use the [OpenTitan IP](https://opentitan.org/book/hw/ip/i2c/index.html).
We hardcode this block to be in host mode, so you can ignore the target functionality, including register `ovrd`, `val`, `target_id`, `acqdata` and `txdata`.
Other than those you can find the [register definitions here](https://opentitan.org/book/hw/ip/i2c/doc/registers.html).
The registers `0x00` - `0x10` are also not accessible.
The control register is just hardwired to be in host mode.

For Sonata, we include two I2C blocks.
The registers of the second I2C block can be accessed with and additional `0x1000` offset.
These can be connected to any of these I2C targets:
- Two for the QWIIC connectors.
- One for the mikroBUS.
- One for the Raspberry Pi hat.
- One for the Arduino shield.
