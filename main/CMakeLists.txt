idf_component_register(SRCS "pms5003.c"
                INCLUDE_DIRS "."
                )
set_source_files_properties("pms5003.c"
        PROPERTIES COMPILE_FLAGS
        -Wall -Wextra -Werror
)