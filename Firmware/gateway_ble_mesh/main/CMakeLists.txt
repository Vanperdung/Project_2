idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES ${project_dir}/certs/git_ota.pem)
