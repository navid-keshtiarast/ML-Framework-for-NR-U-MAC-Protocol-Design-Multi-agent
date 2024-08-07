set(source_files
    model/nr-on-off-access-manager.cc
    model/nr-lbt-access-manager.cc
    model/application-scenario.cc
    model/interference-application.cc
    model/l2-setup.cc
    model/no-interference-node-distribution.cc
    model/node-distribution-scenario.cc
    model/nr-single-bwp-setup.cc
    model/single-pair-node-scenario.cc
    model/varying-interference-node-distribution.cc
    model/in-front-node-distribution.cc
    model/file-output-manager.cc
    model/half-matrix-layout.cc
    model/wifi-setup.cc
    helper/nr-u-trace-helper.cc
)

set(header_files
    model/nr-on-off-access-manager.h
    model/nr-lbt-access-manager.h
    model/application-scenario.h
    model/interference-application.h
    model/in-front-node-distribution.h
    model/l2-setup.h
    model/no-interference-node-distribution.h
    model/node-distribution-scenario.h
    model/nr-single-bwp-setup.h
    model/single-pair-node-scenario.h
    model/varying-interference-node-distribution.h
    model/wigig-setup.h
    model/output-manager.h
    model/file-output-manager.h
    model/half-matrix-layout.h
    model/wifi-setup.h
    helper/nr-u-trace-helper.h
)


set(test_sources

)

build_lib(
  LIBNAME nr-u
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnr}
    ${libspectrum}
    ${libantenna}
    ${libwifi}
  TEST_SOURCES ${test_sources}
)

