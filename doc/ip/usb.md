# USB 2.0 device

The USB (Universal Serial Bus) device hardware IP block in Sonata is taken from the OpenTitan project.
You can find the full documentation [here](https://opentitan.org/book/hw/ip/usbdev/index.html).

Multiple USB devices are allowed to be instantiated at a `0x1000` offset from each other up to a maximum of 256 instantiations.
