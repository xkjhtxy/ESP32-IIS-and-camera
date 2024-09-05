# idf_component_register(SRCS "main.c" "mServer.c" "mWifi.c" "Queue.c"
#                     INCLUDE_DIRS ".")
file(GLOB source_file ./*.c)
idf_component_register(SRCS "${source_file}"
                    INCLUDE_DIRS .)
