# Ibex

For details on how Ibex works please look at the vendored in documentation.
This page highlights the changes that we made to Ibex for the Sonata system.

The Sonata board has a CHERI enabled LED which is hard wired to the value of `cheri_pmode_i` of the Ibex core module.

## Capability exception LEDs

In the [CHERIoT specification](https://www.microsoft.com/en-us/research/uploads/prod/2023/02/cheriot-63e11a4f1e629.pdf) there are number of capability exception codes.
The Sonata board has dedicated LEDs to indicate any of these errors.
When one of these exceptions is seen, it gets latched and displayed on the LED.
The LEDs do not clear once the exception is handled, instead software needs to clear these LEDs through a special CSR.
Custom M-mode CSR `0xBC0` is used for this purpose.
If bit 0 is set to 1 then hardware no longer drives the LEDs and software is in full control over the output.
The other bits positions correspond to the value of the exception code.

| Bit offset | Description |
|------------|-------------|
| 24         | Permit access system registers violation |
| 22         | Permit store local capability violation |
| 21         | Permit store capability violation |
| 19         | Permit store violation |
| 18         | Permit load violation |
| 17         | Permit execute violation |
| 3          | Seal violation |
| 2          | Tag violation |
| 1          | Bounds violation |
| 0          | Disable     |
