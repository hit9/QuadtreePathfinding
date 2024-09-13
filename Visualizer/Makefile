default: build

install:
	conan install . --output-folder=Build --build=missing -s compiler.cppstd=20

cmake:
	@if [ ! -d Build ]; then \
		$(MAKE) install; \
	fi
	cd Build && cmake .. \
        -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=1

build:
	@if [ ! -d Build ]; then \
		$(MAKE) cmake; \
	fi
	make -C Build

.PHONY: build

