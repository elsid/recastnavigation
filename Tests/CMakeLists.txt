add_executable(Tests
	Detour/Tests_Detour.cpp
	Recast/Tests_Recast.cpp
	Integration/Issue729.cpp
)

set_property(TARGET Tests PROPERTY CXX_STANDARD 17)
set_property(TARGET Tests PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET Tests PROPERTY CXX_EXTENSIONS OFF)

target_compile_definitions(Tests PRIVATE
	RECASTNAVIGATION_TESTS_INTEGRATION_MESHES=u8"${PROJECT_SOURCE_DIR}/RecastDemo/Bin/Meshes")

target_link_libraries(Tests Recast Detour DebugUtils)

find_package(Catch2 QUIET)
if (Catch2_FOUND)
	target_link_libraries(Tests Catch2::Catch2WithMain)
else()
	include_directories(./Contrib)
	target_sources(Tests PRIVATE Contrib/catch2/catch_amalgamated.cpp)
endif()

add_test(Tests Tests)
