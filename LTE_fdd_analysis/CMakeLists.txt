include(GrPlatform)
include_directories(hdr
  ${CMAKE_SOURCE_DIR}/liblte/hdr
  ${CMAKE_SOURCE_DIR}/libtools/hdr
  ${CMAKE_SOURCE_DIR}/cmn_hdr
)
add_executable(LTE_Fdd_Analysis
  src/LTE_Fdd_Analysis.cc
)

target_link_libraries(LTE_Fdd_Analysis lte fftw3f tools pthread ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES} ${GNURADIO_FILTER_LIBRARIES} ${GNURADIO_PMT_LIBRARIES} ${GNURADIO_OSMOSDR_LIBRARIES})
set_target_properties(LTE_Fdd_Analysis PROPERTIES DEFINE_SYMBOL "LTE_fdd_dl_scan_state_machine_EXPORTS")
install(TARGETS LTE_Fdd_Analysis DESTINATION bin)

