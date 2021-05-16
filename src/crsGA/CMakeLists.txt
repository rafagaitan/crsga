set(header_path ${CMAKE_SOURCE_DIR}/include/crsGA)

set(sources
    Logger.cpp
)

SET(headers
    ${header_path}/Export.hpp
    ${header_path}/Logger.hpp
    ${header_path}/ThreadSafeQueue.hpp
    ${header_path}/ThreadPool.hpp
    ${header_path}/Common.hpp
    ${header_path}/ScopedThread.hpp
    ${header_path}/GeneticAlgorithm.hpp
    ${header_path}/SimulatedAnnealing.hpp
)

add_library(crsGA STATIC ${sources} ${headers})
target_include_directories(crsGA
                           PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
set_target_properties(crsGA 
                      PROPERTIES PUBLIC_HEADER "${headers}")
target_link_libraries(crsGA
                      PUBLIC ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS crsGA
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/crsGA)


