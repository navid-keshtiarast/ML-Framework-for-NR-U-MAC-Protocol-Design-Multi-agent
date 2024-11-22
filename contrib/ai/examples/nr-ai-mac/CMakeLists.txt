build_lib_example(
  NAME ns3ai_nrmac_gym
        SOURCE_FILES
            use-gym/nr-ai-mac.cc
            use-gym/nr-mac-env.cc
        LIBRARIES_TO_LINK
            ${libai}
            ${libcore}
            ${libpoint-to-point}
            ${libpoint-to-point-layout}
            ${libnetwork}
            ${libapplications}
            ${libmobility}
            ${libcsma}
            ${libinternet}
            ${libspectrum}
            ${libwifi}
            ${libflow-monitor}
            ${libvr-app}
            ${libnr}
            ${libnr-u}
)

# build_lib_example(
#         NAME ns3ai_nrmac_msg
#         SOURCE_FILES
#             use-msg/nr-ai-mac.cc
#             use-msg/nr-mac-env.cc
#         LIBRARIES_TO_LINK
#             ${libai}
#             ${libcore}
#             ${libpoint-to-point}
#             ${libpoint-to-point-layout}
#             ${libnetwork}
#             ${libapplications}
#             ${libmobility}
#             ${libcsma}
#             ${libinternet}
#             ${libspectrum}
#             ${libwifi}
#             ${libflow-monitor}
#             ${libvr-app}
#             ${libnr}
#             ${libnr-u}
# )

# pybind11_add_module(ns3ai_nrmac_msg_py use-msg/nr_mac_py.cc)
# set_target_properties(ns3ai_nrmac_msg_py PROPERTIES
#         LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/use-msg)
# target_link_libraries(ns3ai_nrmac_msg_py PRIVATE ${libai})

# # Build Python interface along with C++ lib
# add_dependencies(ns3ai_nrmac_msg ns3ai_nrmac_msg_py)