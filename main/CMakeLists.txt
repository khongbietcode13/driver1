idf_component_register(SRCS "audio_frontend.c" "main.c" "bmi160.c"
                       INCLUDE_DIRS ".",
                       REQUIRES DRIVER bmi160,
                       REQUIRES DRIVER audio_frontend
                       
                       )