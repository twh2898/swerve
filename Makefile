
FORMAT_FILES=controllers/swerve libraries/base libraries/util
FIND_CMD=find $(FORMAT_FILES) -name '*.cpp' -or -name '*.hpp'
CLANG_CMD=$(FIND_CMD) | xargs clang-format --Werror --sort-includes

setup:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

build:
	cmake --build build

lint:
	@$(CLANG_CMD) --dry-run

format:
	@$(CLANG_CMD) -i

clean:
	rm -rf build/ controllers/swerve/swerve controllers/swerve/swerve.log

.PHONY: setup build lint format
