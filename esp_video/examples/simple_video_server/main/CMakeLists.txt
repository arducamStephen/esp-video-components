set(srcs "simple_video_server_example.c" "app_isp.c" "AI/app_drawing_utils.c")

set(include_dirs "AI")
set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_INCLUDE_DIRS .
                       EMBED_TXTFILES ${html_files})
