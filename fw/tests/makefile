TESTS = serialization_test

.PHONY: test
	test: $(TESTS)
		@for t in $(TESTS); do \
		    		echo ">> running $$t"; \
						./$$t | ./greatest/greenest; \
							done

.PHONY: clean
	clean:
		rm -f $(TESTS)

serialisation_test: log_test.c ../core/log.c log_mock_fun.c ../lib/cmp/cmp.c ../lib/cmp_mem_access/cmp_mem_access.c
		gcc -Wall -I../ -DLOG_TEST=1 log_test.c ../core/log.c log_mock_fun.c ../lib/cmp/cmp.c ../lib/cmp_mem_access/cmp_mem_access.c -o $@
