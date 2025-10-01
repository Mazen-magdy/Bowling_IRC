idf_component_register(SRCS "main.c" 
                       "../Project_components/PID.c"
                       "../Project_components/Motor.c" 
                       "../Project_components/Encoder.c"
                       "../Project_components/Serial.c"
                    INCLUDE_DIRS "." "../Project_components")