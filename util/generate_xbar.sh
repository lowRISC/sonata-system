mkdir -p rtl/bus
vendor/lowrisc_ip/util/tlgen.py -t data/xbar_main.hjson -o rtl/bus
vendor/lowrisc_ip/util/tlgen.py -t data/xbar_ifetch.hjson -o rtl/bus
rm -r rtl/bus/data
rm -r rtl/bus/dv
mv rtl/bus/rtl/autogen/*.sv rtl/bus
rm -r rtl/bus/rtl
