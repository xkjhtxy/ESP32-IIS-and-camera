file(GLOB source_file ./*.c)
idf_component_register(SRCS "${source_file}"
                    INCLUDE_DIRS .)
