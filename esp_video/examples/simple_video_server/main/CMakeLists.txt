<<<<<<< HEAD
set(srcs 
        "simple_video_server_example.c" 
        "app_isp.c" 
        "app_drawing_utils.c"
        "ArducamIMX500SDK.cc"
        "utils.cc"
        )

set(incs 
        "."
        "../flatbuffers/include")

=======
set(srcs "simple_video_server_example.c" 
         "app_isp.c" 
         "AI/app_drawing_utils.c"
         "AI/esp_painter.c")

set(include_dirs "AI")
file(GLOB_RECURSE FONT_SRCS AI/font/*.c)
>>>>>>> origin/add_arducam_pivariety
set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

<<<<<<< HEAD
idf_component_register(SRCS "${srcs}"
                       PRIV_INCLUDE_DIRS "${incs}"
=======
idf_component_register(SRCS "${srcs}" "${FONT_SRCS}"
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_INCLUDE_DIRS .
>>>>>>> origin/add_arducam_pivariety
                       EMBED_TXTFILES ${html_files})
