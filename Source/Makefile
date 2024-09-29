default: build

cmake:
	cmake -B Build -S .

build:
	@if [ ! -d Build ]; then \
		$(MAKE) cmake; \
	fi
	make -C Build

.PHONY: build
