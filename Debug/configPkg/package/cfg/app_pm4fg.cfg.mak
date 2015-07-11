# invoke SourceDir generated makefile for app.pm4fg
app.pm4fg: .libraries,app.pm4fg
.libraries,app.pm4fg: package/cfg/app_pm4fg.xdl
	$(MAKE) -f /home/vagrant/workspace_v6_1/Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP/src/makefile.libs

clean::
	$(MAKE) -f /home/vagrant/workspace_v6_1/Octanis1-Mainboard-Firmware_MSP_EXP432P401RLP/src/makefile.libs clean

