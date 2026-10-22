# Reset the warning level for third parties
function(vor_reset_warning_level)
    remove_definitions(/W4)
    add_definitions(/W3)
    add_definitions(/wd4245)
    add_definitions(/wd4389)
endfunction()

vor_reset_warning_level()

add_definitions(-D_SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS)

aux_source_directories(SOURCES "Source Files"               .)
aux_source_directories(SOURCES "Source Files\\LM6"          LM7)
aux_source_directories(SOURCES "Source Files\\rply"         rply)
aux_source_directories(SOURCES "Source Files\\shewchuk"     shewchuk)
aux_source_directories(SOURCES "Source Files\\zlib"         zlib)
aux_source_directories(SOURCES "Source Files\\gzstream"     gzstream)
aux_source_directories(SOURCES "Source Files\\PoissonRecon" PoissonRecon)
aux_source_directories(SOURCES "Source Files\\xatlas"       xatlas)

add_library(point2mesh_third_party OBJECT ${SOURCES})

set_target_properties(
    point2mesh_third_party PROPERTIES
    FOLDER "3rd_party"
)