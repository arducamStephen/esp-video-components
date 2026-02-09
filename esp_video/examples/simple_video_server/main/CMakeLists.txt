set(srcs 
        "simple_video_server_example.c" 
        "app_isp.c" 
        "app_drawing_utils.c"
        "ArducamIMX500SDK.cc"
        "utils.cc"
        "esp_painter.c"
        )

set(incs 
        "."
        "../flatbuffers/include")

file(GLOB_RECURSE FONT_SRCS font/*.c)

set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

idf_component_register(SRCS "${srcs}" "${FONT_SRCS}"
                       INCLUDE_DIRS ${incs}
                       PRIV_INCLUDE_DIRS .
                       EMBED_TXTFILES ${html_files})
