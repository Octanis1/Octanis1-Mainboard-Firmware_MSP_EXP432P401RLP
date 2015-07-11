#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/vagrant/ti/tirtos_msp43x_2_12_00_24/packages;/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/bios_6_41_03_51/packages;/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/uia_2_00_02_39/packages;/home/vagrant/ti/ccsv6/ccs_base
override XDCROOT = /home/vagrant/ti/xdctools_3_31_00_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/vagrant/ti/tirtos_msp43x_2_12_00_24/packages;/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/bios_6_41_03_51/packages;/home/vagrant/ti/tirtos_msp43x_2_12_00_24/products/uia_2_00_02_39/packages;/home/vagrant/ti/ccsv6/ccs_base;/home/vagrant/ti/xdctools_3_31_00_24_core/packages;..
HOSTOS = Linux
endif
