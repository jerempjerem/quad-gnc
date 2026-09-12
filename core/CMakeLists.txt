add_library(core STATIC src/version.cpp)
target_include_directories(core PUBLIC include)
target_compile_options(core PRIVATE -Wall -Wextra -Wpedantic)

add_subdirectory(tests)

