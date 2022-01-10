cmake_minimum_required(VERSION 3.18)
set(PROJECT_NAME FbxParser)

set(COMPILER_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(COMPILER_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})

OPTION(FBX_VENDER_DIR "Optional Fbx SDK path." "")
OPTION(USE_IMAGE_SHARED "Use image as shared lib." OFF)

message("${FBX_VENDER_DIR}")

set(OSX_VERSION "10.9")

if (WIN32)
  project(${PROJECT_NAME} C CXX ASM)
  set (CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
  add_definitions(-DUNICODE)
  add_definitions(-D_UNICODE)
endif()
if (APPLE)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "${OSX_VERSION}" CACHE STRING "Minimum OS X deployment version")
  project(${PROJECT_NAME} C CXX OBJC ASM)
  set(CMAKE_C_FLAGS "-fobjc-arc")
  set(CMAKE_CXX_FLAGS "-std=c++14 -stdlib=libc++ -fobjc-arc")
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC "YES")
  set(CMAKE_MACOSX_RPATH ON)
  set(BUILD_RPATH_USE_ORIGIN TRUE)
  set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(CMAKE_BUILD_TYPE MATCHES DEBUG OR CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO) 
elseif(CMAKE_BUILD_TYPE MATCHES RELEASE OR CMAKE_BUILD_TYPE MATCHES MINSIZEREL)
else()
set(CMAKE_BUILD_TYPE "MINSIZEREL")
endif()

file (
  GLOB_RECURSE fbx_src
  LIST_DIRECTORIES false
  "${PROJECT_SOURCE_DIR}/Source/*.cpp"
  "${PROJECT_SOURCE_DIR}/Source/*.h"
)

foreach(_source IN ITEMS ${fbx_src})
  if (IS_ABSOLUTE "${_source}")
    file(RELATIVE_PATH _source_rel "${PROJECT_SOURCE_DIR}" "${_source}")
  else()
    set(_source_rel "${_source}")
  endif()
  get_filename_component(_source_path "${_source_rel}" PATH)
  if (WIN32)
    string(REPLACE "/" "\\" _source_path_replaced "${_source_path}")
  else()
    set(_source_path_replaced "${_source_path}")
  endif()
  source_group("${_source_path_replaced}" FILES "${_source}")
endforeach()

set(LIBRARY_NAME "fbx")

add_library(${LIBRARY_NAME} SHARED ${fbx_src})

add_subdirectory(ThirdParty)

if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819")
endif()

target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/ThirdParty/ForsythTriOO")
target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/ThirdParty/ext-mikktspace")
target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/ThirdParty/glm")
target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/ThirdParty/spdlog/include")
target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/ThirdParty/filesystem/include")
target_include_directories(${LIBRARY_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/Source")
target_link_libraries(${LIBRARY_NAME} "ForsythTriOO")
target_link_libraries(${LIBRARY_NAME} "ext-mikktspace")
if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819")
  if(USE_IMAGE_SHARED)
    if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
      target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
      target_link_libraries(${LIBRARY_NAME} "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/lib/image.lib")
    else()
      target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
      target_link_libraries(${LIBRARY_NAME} "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/lib/image.lib")
    endif()
  else()
    target_compile_definitions(${LIBRARY_NAME} PRIVATE IMAGE_STATIC)
    if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
      file(
        GLOB_RECURSE image_debug_libs
        LIST_DIRECTORIES false
        "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/static/Debug/lib/*.lib"
      )
      file(
        GLOB_RECURSE image_release_libs
        LIST_DIRECTORIES false
        "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/static/MinSizeRel/lib/*.lib"
      )
      target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/static/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
      target_link_libraries(${LIBRARY_NAME} "$<IF:$<CONFIG:Debug>,${image_debug_libs},${image_release_libs}>")
    else()
      file(
        GLOB_RECURSE image_debug_libs
        LIST_DIRECTORIES false
        "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/static/Debug/lib/*.lib"
      )
      file(
        GLOB_RECURSE image_release_libs
        LIST_DIRECTORIES false
        "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/static/MinSizeRel/lib/*.lib"
      )
      target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/static/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
      target_link_libraries(${LIBRARY_NAME} "$<IF:$<CONFIG:Debug>,${image_debug_libs},${image_release_libs}>")
    endif()
  endif()
else()
  if(USE_IMAGE_SHARED)
    target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
    target_link_libraries(${LIBRARY_NAME} "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/lib/libimage.dylib")
  else()
    target_compile_definitions(${LIBRARY_NAME} PRIVATE IMAGE_STATIC)
    file(
      GLOB_RECURSE image_debug_libs
      LIST_DIRECTORIES false
      "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/static/Debug/lib/*.a"
    )
    file(
      GLOB_RECURSE image_release_libs
      LIST_DIRECTORIES false
      "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/static/MinSizeRel/lib/*.a"
    )
    target_include_directories(${LIBRARY_NAME} PRIVATE "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/static/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/include")
    target_link_libraries(${LIBRARY_NAME} "$<IF:$<CONFIG:Debug>,${image_debug_libs},${image_release_libs}>")
  endif()
    
  target_compile_options(${LIBRARY_NAME} PUBLIC -fvisibility=hidden)
  target_compile_options(${LIBRARY_NAME} PUBLIC -fvisibility-inlines-hidden)
endif()

# Search for FBX sdk package
if("${FBX_VENDER_DIR}" STREQUAL "")
  unset(FBX_VENDER_DIR)
else()
  if (WIN32)
    string(REPLACE "/" "\\" FBX_VENDER_DIR_REPLCAED "${FBX_VENDER_DIR}")
  else()
    set(FBX_VENDER_DIR_REPLCAED "${FBX_VENDER_DIR}")
  endif()
  set(FBX_DIR "${FBX_VENDER_DIR}/*/")
  message(${FBX_DIR})
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(Fbx 2020.2.1)
if(NOT FBX_FOUND)
    message("Fbx SDK not found!")
else()
  target_include_directories(${LIBRARY_NAME} PUBLIC ${FBX_INCLUDE_DIRS})
  target_link_libraries(${LIBRARY_NAME} debug ${FBX_LIBRARIES_DEBUG} optimized ${FBX_LIBRARIES})
endif()
target_compile_definitions(${LIBRARY_NAME} PRIVATE FBX_DLL)

set(INSTALL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

install(TARGETS ${LIBRARY_NAME} ${LIBRARY_NAME}
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}")

set(LIBRARY_PUBLIC_HDRS 
  ${PROJECT_SOURCE_DIR}/Source/FbxParser.h
)

install(FILES ${LIBRARY_PUBLIC_HDRS} DESTINATION "${INSTALL_INC_DIR}")
if(WIN32)
  install(FILES $<TARGET_PDB_FILE:${LIBRARY_NAME}> DESTINATION bin OPTIONAL)
endif()

# ============ Test ==============

file (
  GLOB_RECURSE main_src
  LIST_DIRECTORIES false
  "${PROJECT_SOURCE_DIR}/Test/*.cpp"
  "${PROJECT_SOURCE_DIR}/Test/*.h"
)

foreach(_source IN ITEMS ${main_src})
  if (IS_ABSOLUTE "${_source}")
    file(RELATIVE_PATH _source_rel "${PROJECT_SOURCE_DIR}" "${_source}")
  else()
    set(_source_rel "${_source}")
  endif()
  get_filename_component(_source_path "${_source_rel}" PATH)
  if (WIN32)
    string(REPLACE "/" "\\" _source_path_replaced "${_source_path}")
  else()
    set(_source_path_replaced "${_source_path}")
  endif()
  source_group("${_source_path_replaced}" FILES "${_source}")
endforeach()

add_executable(${PROJECT_NAME} ${main_src})
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/ThirdParty/rapidjson/include")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/ThirdParty/cxxopts/include")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/ThirdParty/filesystem/include")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/ThirdParty/glm")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/Test")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/Source")
if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819")
endif()
target_link_libraries(${PROJECT_NAME} ${LIBRARY_NAME})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_FILE_DIR:${LIBRARY_NAME}>" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
)

if(WIN32)
  if(USE_IMAGE_SHARED)
    if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
      add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x86/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/bin" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
      )
    else()
      add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/download/ImageDecoder/win32/x64/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/bin" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
      )
    endif()
  endif()
else()
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath,./")
  set_target_properties(${PROJECT_NAME} PROPERTIES CMAKE_BUILD_RPATH "$ORIGIN")
  set_target_properties(${PROJECT_NAME} PROPERTIES CMAKE_INSTALL_RPATH "$ORIGIN")
  if(USE_IMAGE_SHARED)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/download/ImageDecoder/darwin/shared/$<IF:$<CONFIG:Debug>,Debug,MinSizeRel>/lib" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )
  endif()
endif()