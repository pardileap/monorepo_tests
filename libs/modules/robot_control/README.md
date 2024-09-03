# robot_control

Install the repo
```
git clone --recurse-submodules -j8 https://github.com/LeapAutomation/robot_control.git
```

# Makefile Options

The makefile provides a series of commands to simplify the building and formatting of the code:

- *build*: builds the project in the folder called `build/`
- *format*: format the code following the `.clang-format` rules
- *tidy*: check the code against the rules setup in `.clang-tidy`
- *tidy-fix*: check the code against the rules setup in `.clang-tidy` and tries to fix the inconsitencies
- *clean*: removes the `build/` folder and all its contents
- *check*: runs *format* and *tidy*
- *check-fix*: runs *format* and *tidy-fix*
