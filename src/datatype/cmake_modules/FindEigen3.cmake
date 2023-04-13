# Try to find eigen3.

if ( EIGEN3_INCLUDE_DIR )
    set( EIGEN3_INCLUDE_DIR )
else ( EIGEN3_INCLUDE_DIR )
    find_path( EIGEN3_INCLUDE_DIR NAMES Eigen/Core
        PATH_SUFFIXES eigen3/
        HINTS         ${INCLUDE_INSTALL_DIR}
                      /usr/local/include
                      ${KDE4_INCLUDE_DIR}
    )
endif ( EIGEN3_INCLUDE_DIR )
