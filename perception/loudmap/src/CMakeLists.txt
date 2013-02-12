SET (loudmap_SRCS
  LoudTree.cpp
  LoudTreeNode.cpp
  )

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( loudmap SHARED ${loudmap_SRCS})
ADD_LIBRARY( loudmap-static STATIC ${loudmap_SRCS})
SET_TARGET_PROPERTIES(loudmap-static PROPERTIES OUTPUT_NAME "loudmap") 

TARGET_LINK_LIBRARIES(loudmap)

ADD_SUBDIRECTORY( testing )

#ADD_EXECUTABLE(simpleLoud simple.cpp)
#TARGET_LINK_LIBRARIES(simpleLoud loudmap)


# installation:
# store all header files to install:
# file(GLOB loudmap_impl_HDRS *.h *.hxx *.hpp)
# install(FILES ${loudmap_impl_HDRS} DESTINATION include/loudmap)

install(TARGETS 
	loudmap
	loudmap-static 
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

