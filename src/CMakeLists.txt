# ── the library itself ──────────────────────────────────────────────────────
add_library(mw
  version.cpp
  time.cpp
  message.cpp
  transport.cpp          # <-- add this
)
add_library(mw::mw ALIAS mw)

target_include_directories(mw
  PUBLIC  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(mw
  PRIVATE
    MW_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    MW_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    MW_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    MW_VERSION_STRING="${PROJECT_VERSION}"
)

target_link_libraries(mw
  PRIVATE mw_warnings          # our discipline, not our users'
  PUBLIC  mw_sanitizers        # must match across the whole program
  PUBLIC  mw_proto             # our public headers include the generated ones
  PRIVATE mw_zeromq            # <-- add this
)