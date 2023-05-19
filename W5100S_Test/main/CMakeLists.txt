set(srcs "W5100S_Test_main.c")

idf_component_register(SRCS ${srcs}
                            "Application/loopback/loopback.c"
                            "W5100S/w5100s.c"
                            "W5500/w5500.c"
                            "wizchip_conf.c"
                            "socket.c"
                    INCLUDE_DIRS ".")
