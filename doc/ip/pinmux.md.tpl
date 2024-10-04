${"#"} Pin multiplexer

This allows software to dynamically switch FPGA pins between input and output as well as reassign them for SPI, I2C, UART, etc.
The block also allows pad control.

To see the possible mappings, please refer to [the top configuration](https://github.com/lowRISC/sonata-system/blob/main/data/top_config.toml).
All selectors are byte addressable, this means that you can write four selectors at a time with a 32-bit write.

There are output pin selectors, which select which block output is connected to a particular FPGA pin.
The selector is one-hot, so you need to write `8'b100` if you want to select input 3 for example.
The default value for all of these selectors is `'b10`.

| Address | Pin output | Possible block outputs |
|---------|------------|------------------------|
% for output_idx, (pin_output, idx_str, idx_alt, possible_blocks) in enumerate(output_list):
| ${f"{output_idx:#0{5}x}"} | ${pin_output}${idx_alt} | 0${"".join([", " + block + "_" + io + "_i(" +str(inst) + ")" + bit_str.replace("[", "(").replace("]", ")") for block, io, inst, bit_str, _ in possible_blocks])} |
% endfor

Besides the output pin selectors, there are also selectors for which pin should drive block inputs:

| Address | Block input | Possible pin inputs |
|---------|-------------|---------------------|
% for input_idx, (block_input, inst, bit_idx, bit_str, possible_pins) in enumerate(input_list):
| ${f"{(0x800+input_idx):#0{5}x}"} | ${block_input}_o(${inst})${'' if bit_str == '' else '('+str(bit_idx)+')'} | ${"".join([pin + ", " for pin in possible_pins])}|
% endfor

${"##"} Regeneration

If any changes are made to the top configuration, the templates or the bus, you must regenerate the top.
You can do so using the top generation utility, which regenerates the pinmux, the bus and the sonata package which is used by the SystemVerilog generate statements throughout the project.

```sh
./util/top_gen.py
```
