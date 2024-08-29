default: build

cmake:
	cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=1

build:
	make -C build

run:
	./build/quadtree-pathfinding-example-flowfield

.PHONY: build

