obj-m += ads126x.o

all: module dt
	echo Building Devicetree Overlay and kernel module

module:
	make -C ~/linux/$(shell uname -r)/build M=$(PWD) modules
dt: ads1262overlay.dts
	dtc -@ -I dts -O dtb -o ads1262overlay.dtbo ads1262overlay.dts
clean:
	make -C /linux/$(shell uname -r)/build M=$(PWD) clean
	rm -rf ads1262overlay.dtbo