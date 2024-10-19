default: build

cmake:
	cmake -B Build -S . -DCMAKE_CXX_STANDARD=20 -DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=1

build:
	@if [ ! -d Build ]; then \
		$(MAKE) cmake; \
	fi
	make -C Build

.PHONY: build
