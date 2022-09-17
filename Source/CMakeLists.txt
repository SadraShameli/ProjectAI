# Main Source Files
add_subdirectory(src)
target_include_directories(ProjectAI PRIVATE include)

# Add PreCompiled Header
target_precompile_headers(ProjectAI PRIVATE include/CommonHeaders.h)