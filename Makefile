obj-m += ads1262.o

all: module dt
	echo Building Devicetree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: ads1262overlay.dts
	dtc -@ -I dts -O dtb -o ads1262overlay.dtbo ads1262overlay.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf ads1262overlay.dtbo
