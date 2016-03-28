
TESTS = flash_test

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

