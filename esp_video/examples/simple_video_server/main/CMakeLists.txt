set(srcs 
        "simple_video_server_example.c" 
        "app_isp.c" 
        "app_drawing_utils.c"
        "ArducamIMX500SDK.cc"
        )

set(incs 
        "."
        "../flatbuffers/include")

set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

idf_component_register(SRCS "${srcs}"
                       PRIV_INCLUDE_DIRS "${incs}"
                       EMBED_TXTFILES ${html_files})
