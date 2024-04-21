# Find libtorch.
list( APPEND CMAKE_PREFIX_PATH "/mnt/e/libtorch_cpu" )
find_package( Torch REQUIRED )

# Package target lib_pytorch_for_cxx.
aux_source_directory( . AUX_SRC_LIB_TORCH_FOR_CXX )
add_library( lib_pytorch_for_cxx
    ${AUX_SRC_LIB_TORCH_FOR_CXX}
)
target_link_libraries( lib_pytorch_for_cxx
    ${TORCH_LIBRARIES}
)
target_include_directories( lib_pytorch_for_cxx PUBLIC
    .
)
