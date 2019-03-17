add_subdirectory(optimization)
add_subdirectory(path_planning)

# Machine Learning Package
find_package(SWIG)
if(SWIG_FOUND)
    add_subdirectory(machine_learning)
else()
    message("Swig is not installed. machine learning codes will not be compiled")
endif()
