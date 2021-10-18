file (
    GLOB_RECURSE ForsythTriOO_src
    LIST_DIRECTORIES false
    "${PROJECT_SOURCE_DIR}/ThirdParty/ForsythTriOO/*.cpp"
    "${PROJECT_SOURCE_DIR}/ThirdParty/ForsythTriOO/*.c"
    "${PROJECT_SOURCE_DIR}/ThirdParty/ForsythTriOO/*.h"
    )

add_library("ForsythTriOO" STATIC ${ForsythTriOO_src})