# - Find the Boost NumericBindings includes and libraries.

#
# Look into path specified by configuration variable BOOSTNUMERICBINDINGS_DIR
# If BOOSTNUMERICBINDINGS_DIR is not given at configuration,
# look into path specified by environment variable
# BOOSTNUMERICBINDINGS_ROOT
#
# Search can be canceled by directly setting BoostNumericBindings_INCLUDE_DIR.



IF(BOOSTNUMERICBINDINGS_DIR)
  SET(BOOSTNUMERICBINDINGS_DIR_SEARCH ${BOOSTNUMERICBINDINGS_DIR}/include)
ELSE(BOOSTNUMERICBINDINGS_DIR)
  SET(BOOSTNUMERICBINDINGS_DIR_SEARCH $ENV{BOOSTNUMERICBINDINGS_ROOT})
ENDIF(BOOSTNUMERICBINDINGS_DIR)

IF(BOOSTNUMERICBINDINGS_DIR_SEARCH)
  FILE(TO_CMAKE_PATH ${BOOSTNUMERICBINDINGS_DIR_SEARCH} BOOSTNUMERICBINDINGS_DIR_SEARCH)
ENDIF(BOOSTNUMERICBINDINGS_DIR_SEARCH)

IF(WIN32)
  LIST(APPEND BOOSTNUMERICBINDINGS_DIR_SEARCH
       "C:/boost-sandbox/include" "D:/boost-sandbox/include")
ENDIF(WIN32)

# Look for an installation.
SET(BoostNumericBindings_FOUND 0)

FIND_PATH(BoostNumericBindings_INCLUDE_DIR
  NAMES boost/numeric/bindings/traits/config.hpp
  PATH_SUFFIXES ${SUFFIX_FOR_PATH}
  PATHS ${BOOSTNUMERICBINDINGS_DIR_SEARCH}
  DOC "The directory containing the boost bindings for lapack"
)

IF(BoostNumericBindings_INCLUDE_DIR)
  SET(BoostNumericBindings_FOUND 1)
ENDIF(BoostNumericBindings_INCLUDE_DIR)


