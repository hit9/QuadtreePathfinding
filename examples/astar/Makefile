default: build

cmake:
	cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=1

build:
	@if [ ! -d build ]; then \
		$(MAKE) cmake; \
	fi
	make -C build

run:
	./build/quadtree-pathfinding-example-astar

.PHONY: build

