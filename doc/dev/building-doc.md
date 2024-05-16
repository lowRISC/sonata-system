# Building Documentation

The documentation uses [mdBook](https://rust-lang.github.io/mdBook/) see the [installation guide](https://rust-lang.github.io/mdBook/guide/installation.html) for further details on installation.

Once mdBook is installed the documentation can be built and viewed with:

```bash
mdbook serve --open
# Avoid FuseSoC using copied files in the book directory
touch book/FUSESOC_IGNORE
```

The second line can be ignored if you aren't building the FPGA bitstream (which uses fusesoc).

## Windows Quick-Start

On Windows the easiest installation method is to copy the precompiled `mdbook.exe` available as a release on the previous link to the sonata-system root directory.

```bash
./mdbook.exe serve --open
```