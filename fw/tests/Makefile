
TESTS = flash_test serialization_test

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

serialization_test: log_test.c ../core/log.c log_mock_fun.c ../lib/cmp/cmp.c ../lib/cmp_mem_access/cmp_mem_access.c
	gcc -Wall -I../ -DLOG_TEST=1 log_test.c ../core/log.c log_mock_fun.c ../lib/cmp/cmp.c ../lib/cmp_mem_access/cmp_mem_access.c -o $@
