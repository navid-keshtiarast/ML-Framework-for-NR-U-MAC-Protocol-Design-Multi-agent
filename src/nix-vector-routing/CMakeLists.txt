build_lib(
  LIBNAME nix-vector-routing
  SOURCE_FILES model/ipv4-nix-vector-routing.cc
               helper/ipv4-nix-vector-helper.cc
  HEADER_FILES helper/ipv4-nix-vector-helper.h
               model/ipv4-nix-vector-routing.h
  LIBRARIES_TO_LINK ${libinternet}
)
