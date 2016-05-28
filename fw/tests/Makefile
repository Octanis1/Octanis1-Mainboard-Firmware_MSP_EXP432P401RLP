
TESTS = flash_test log_test navigation_test pid_test

.PHONY: test
test: $(TESTS)
	@for t in $(TESTS); do \
		echo ">> running $$t"; \
		./$$t | ./greatest/greenest; \
	done

.PHONY: clean
clean:
	rm -f $(TESTS)

flash_test: flash_test.c ../peripherals/flash.c
	gcc -Wall -I../ -DFLASH_TEST=1 flash_test.c ../peripherals/flash.c -o $@

navigation_test: navigation_test.c ../peripherals/navigation.c
	gcc -Wall -std=c99 -I../ -DNAVIGATION_TEST=1 navigation_test.c ../peripherals/navigation.c -lm -o $@

log_test: log_test.c ../core/log.c
	gcc -Wall -I../ -I../lib -DLOG_TEST=1 log_test.c ../core/log.c ../lib/crc8.c ../lib/cmp/cmp.c ../lib/cmp_mem_access/cmp_mem_access.c -o $@

pid_test: pid_test.c ../peripherals/pid.c
	gcc -Wall -I../ -I../lib -DPID_TEST=1 pid_test.c ../peripherals/pid.c  -o $@
