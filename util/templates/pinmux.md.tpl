${"#"} Pin multiplexer

This allows software to dynamically switch FPGA pins between input and output as well as reassign them for SPI, I2C, UART, etc.
The block also allows pad control.

To see the possible mappings, please refer to [the top configuration](https://github.com/lowRISC/sonata-system/blob/main/data/top_config.toml).
All selectors are byte addressable, this means that you can write four selectors at a time with a 32-bit write.

There are output pin selectors, which select which block output is connected to a particular FPGA pin.
The selector is one-hot, so you need to write `8'b100` if you want to select input 3 for example.
The default value for all of these selectors is `'b10`. As a consequence, you will need to use the pinmux before attempting use the additional headers as GPIO (e.g. the Raspberry Pi header's GPIO).

| Address | Pin output | Possible block outputs |
|---------|------------|------------------------|
% for output_idx, (pin, possible_block_outputs, num_options) in enumerate(output_pins):
| ${f"{output_idx:#0{5}x}"} | `${pin.doc_name}` | 0, ${", ".join((f"`{block_io.doc_name}`" for block_io in possible_block_outputs))} |
% endfor

Besides the output pin selectors, there are also selectors for which pin should drive block inputs:

| Address | Block input | Possible pin inputs |
|---------|-------------|---------------------|
% for input_idx, (block_io, possible_pins, num_options) in enumerate(output_block_ios):
| ${f"{(0x800+input_idx):#0{5}x}"} | `${block_io.doc_name}` | ${block_io.default_value}${"".join((f", `{pin.doc_name}`" for pin in possible_pins)) if len(possible_pins) > 0  else f", {block_io.default_value}"} |
% endfor

${"##"} Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
