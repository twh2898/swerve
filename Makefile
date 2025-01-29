
setup:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

build:
	cmake --build build

.PHONY: setup build
