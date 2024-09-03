SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
BUILD_TYPE = Release
PKG_NAME = robot_control

all: build

.PHONY: build
build:
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	@cmake --build $(BUILD_DIR)

.PHONY: run
run:
	@./$(BUILD_DIR)/$(PKG_NAME)_app	

.PHONY: format
format:
	@find $(SRC_DIR) $(INC_DIR) -name '*.*pp' -o -name '*.h*' | xargs clang-format -i 

.PHONY: tidy
tidy:
	@find $(SRC_DIR) $(INC_DIR) -name '*.*pp' -o -name '*.h*' | xargs -r clang-tidy -p=$(BUILD_DIR) --warnings-as-errors='*' 

.PHONY: tidy-fix
tidy-fix:
	@find $(SRC_DIR) $(INC_DIR) -name '*.cpp' -o -name '*.h' | xargs -r clang-tidy -fix -p=$(BUILD_DIR) 

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)

.PHONY: check
check: format tidy

.PHONY: check-fix
check: format tidy-fix

.PHONY: test
test:
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTS=ON
	@cmake --build $(BUILD_DIR)
	@ctest --test-dir $(BUILD_DIR)/test

.PHONY: help
help:
	@echo 	\
	"Makefile Options \n\
	The makefile provides a series of commands to simplify the building and formatting of the code: \n \
	- build: builds the project in the folder called build \n\
	- format: format the code following the .clang-format rules \n\
	- tidy: check the code against the rules setup in .clang-tidy \n\
	- tidy-fix: check the code against the rules setup in .clang-tidy and tries to fix the inconsitencies \n\
	- clean: removes the build folder and all its contents \n\
	- check: runs format and tidy \n\
	- check-fix: runs format and tidy-fix"