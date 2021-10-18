UpdateNewestExternalLib("ext-mikktspace" "https://github.com/tcoppex/ext-mikktspace.git")

file (
    GLOB_RECURSE mikktspace_src
    LIST_DIRECTORIES false
    "${PROJECT_SOURCE_DIR}/ThirdParty/ext-mikktspace/*.cpp"
    "${PROJECT_SOURCE_DIR}/ThirdParty/ext-mikktspace/*.c"
    "${PROJECT_SOURCE_DIR}/ThirdParty/ext-mikktspace/*.h"
    )

add_library("ext-mikktspace" STATIC ${mikktspace_src})