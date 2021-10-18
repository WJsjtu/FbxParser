UpdateExternalLib("ImageDecoder" https://github.com/WJsjtu/ImageDecoder.git "v0.0.1")

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/download/ImageDecoder)
execute_process(COMMAND curl -L -o ${CMAKE_BINARY_DIR}/download/ImageDecoder/release.zip https://github.com/WJsjtu/ImageDecoder/releases/download/v0.0.1/release.zip)

file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/download/ImageDecoder/release.zip DESTINATION ${CMAKE_BINARY_DIR}/download/ImageDecoder VERBOSE)