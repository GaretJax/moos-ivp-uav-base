#--------------------------------------------------------
# The CMakeLists.txt for:                lib_bhv_constAlt
# Author(s):                    Steve Carter Feujo Nomeny
#--------------------------------------------------------
#                                    BEHAVIOR LIBRARY
#--------------------------------------------------------


# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})




#--------------------------------------------------------
#                                    BHV_ConstantAltitude
#--------------------------------------------------------
ADD_LIBRARY(BHV_ConstantAltitude SHARED 
   BHV_ConstantAltitude.cpp)

TARGET_LINK_LIBRARIES(BHV_ConstantAltitude
   "$<LINK_GROUP:RESCAN,helmivp,behaviors,ivpbuild,logic,ivpcore,bhvutil,mbutil,geometry>"
   ${SYSTEM_LIBS}
)


#--------------------------------------------------------
#                                        BHV_Voronoi_uav
#--------------------------------------------------------
ADD_LIBRARY(BHV_Voronoi_uav SHARED 
   BHV_Voronoi_uav.cpp )
TARGET_LINK_LIBRARIES(BHV_Voronoi_uav
   "$<LINK_GROUP:RESCAN,ufield,helmivp,behaviors,ivpbuild,logic,ivpcore,bhvutil,mbutil,geometry>"
   ${SYSTEM_LIBS} )
