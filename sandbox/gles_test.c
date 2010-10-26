#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <GLES/egl.h>
#include <GLES/gl.h>

#include "video_uvc.h"
#include "readwritejpeg.h"
#include "colordetect.h"

#define TEX_SIZE 512

EGLDisplay g_disp = 0;
EGLContext g_ctx = 0;
EGLSurface g_surf = 0;
int g_running = 1;

void sig_handler(int id)
{
    g_running = 0;
}

int initialize_gles()
{
    EGLConfig cfg = 0;
    EGLint major, minor;
    EGLint num_cfg;

    // get the default display handle
    g_disp = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (EGL_NO_DISPLAY == g_disp) {
        printf("error: unable to get display\n");
        return 0;
    }

    // initialize the EGL library and print version
    if (EGL_FALSE == eglInitialize(g_disp, &major, &minor)) {
        printf("error: failed to initialize EGL\n");
        return 0;
    }
    printf("initialized egl v%d.%d\n", major, minor);

    // bind EGL to the OpenGL ES API
    if (EGL_FALSE == eglBindAPI(EGL_OPENGL_ES_API)) {
        printf("error: failed to bind OpenGL ES API\n");
        return 0;
    }

    // specify a 16-bit RGB-565 color config
    const EGLint KColorRGB565AttribList[] = {
        EGL_RED_SIZE,        5,
        EGL_GREEN_SIZE,      6,
        EGL_BLUE_SIZE,       5,
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES_BIT,
        EGL_NONE
    };

    eglChooseConfig(g_disp, KColorRGB565AttribList, &cfg, 1, &num_cfg);
    if (0 == num_cfg) {
        printf("error: no matching configs found\n");
        return 0;
    }
    printf("found %d matching config(s)\n", num_cfg);

    // create the window surface
    g_surf = eglCreateWindowSurface(g_disp, cfg, (NativeWindowType)0, NULL);
    if (EGL_NO_SURFACE == g_surf) {
        printf("error: failed to create window surface\n");
        return 0;
    }

    // create and bind the rendering context
    g_ctx = eglCreateContext(g_disp, cfg, NULL, NULL);
    if (EGL_NO_CONTEXT == g_ctx) {
        printf("error: failed to create rendering context\n");
        return 0;
    }

    if (EGL_FALSE == eglMakeCurrent(g_disp, g_surf, g_surf, g_ctx)) {
        printf("error: failed to bind rendering context\n");
        return 0;
    }

    return 1;
}

int main(int argc, char *argv[])
{
    uint8_t *jpg_buff = NULL, *rgb_buff = NULL;
    unsigned long buff_sz = 0;
    int width = 320, height = 240, fps = 10;
    video_mode_t mode = { width, height, fps };
    video_data_t vid_data;

    const float rx = 320.0f / 512.0f;
    const float ry = 240.0f / 512.0f;

    GLuint videoTex, vbo;
    float afVertices[] = {
         0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         0.0f,  240.0f,  0.0f,  0.0f,  ry,
         320.0f,  0.0f,  0.0f,  rx,  0.0f,
         320.0f,  240.0f,  0.0f,  rx,  ry,
    };
    unsigned int uiSize = 4 * (sizeof(float) * 5);

    track_color_t color = {
        142, 44, 71,
        30, 30, 30,
        5
    };

    track_coords_t box;
    box.width = 320;
    box.height = 240;

    // register CTRL+C to exit
    signal(SIGINT, sig_handler);

    if (!initialize_gles()) {
        fprintf(stderr, "failed to initialize GLES\n");
        return EXIT_FAILURE;
    }

    if (!video_init("/dev/video0", &mode)) {
        fprintf(stderr, "failed to initialize video subsystem\n");
        return EXIT_FAILURE;
    }

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    glGenTextures(1, &videoTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //glBlendFunc(GL_ONE, GL_SRC_COLOR);
    glViewport(0, 0, 480, 272);
    glGenBuffers(1, &vbo);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, uiSize, afVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrthof(0.0f, 480.0f, 0.0f, 272.0f, 0.0f, 1.0f);
    // glFrustumf(-8.0f, 8.0f, -12.0f, 12.0f, -8.0f, 20.0f); 

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
    glBindTexture(GL_TEXTURE_2D, videoTex);

    GLuint  m_ui32Texture;
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &m_ui32Texture);
    glBindTexture(GL_TEXTURE_2D, m_ui32Texture);
    uint8_t *tex = (uint8_t *)malloc(sizeof(uint8_t) * 4 * TEX_SIZE * TEX_SIZE);
    int i, j;

    for(i = 0; i < TEX_SIZE; ++i) {
        for(j = 0; j < TEX_SIZE; ++j) {
            int offset = i * TEX_SIZE * 4 + j * 4;
            GLuint col = (255L << 24) + ((255L - j * 2) << 16) +
                         ((255L - i) << 8) + (255L - i * 2);
            *((GLuint *)&tex[offset]) = col;
        }
    }
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    while (g_running) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!video_lock(&vid_data, 1)) {
            // video disabled, non-functioning, or frame not ready
            continue;
        }

        // copy the jpeg to our buffer now that we're safely locked
        if (buff_sz < vid_data.length) {
            free(jpg_buff);
            buff_sz = vid_data.length;
            jpg_buff = (uint8_t *)malloc(buff_sz);
        }

        memcpy(jpg_buff, vid_data.data, vid_data.length);
        video_unlock();

        if (0 != jpeg_rd_mem(jpg_buff, buff_sz, &rgb_buff, &width, &height)) {
            fprintf(stderr, "decompressed frame size %zu\n", vid_data.length);

            for (i = 0; i < 240; i++) {
                for (j = 0; j < 320; j++) {
                    int src_off = (240 - i) * 320 * 3 + j * 3;
                    int dst_off = i * TEX_SIZE * 4 + j * 4;
                    tex[dst_off + 0] = rgb_buff[src_off + 0];
                    tex[dst_off + 1] = rgb_buff[src_off + 1];
                    tex[dst_off + 2] = rgb_buff[src_off + 2];
                }
            }

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TEX_SIZE, TEX_SIZE,
                         0, GL_RGBA, GL_UNSIGNED_BYTE, tex);

            glLoadIdentity();
            glBindBuffer(GL_ARRAY_BUFFER, vbo);

            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);

            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glVertexPointer(3, GL_FLOAT, sizeof(float) * 5, 0);
            glTexCoordPointer(2, GL_FLOAT,sizeof(float) * 5, (unsigned char*) (sizeof(float) * 3));
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);

            colordetect_hsl_fp32(rgb_buff, &color, &box);
            if (box.detected) {
                float line_coords[] = {
                    (float)box.x1, 240.0f - box.y1, 0.0f,
                    (float)box.x2, 240.0f - box.y1, 0.0f,
                    (float)box.x2, 240.0f - box.y2, 0.0f,
                    (float)box.x1, 240.0f - box.y2, 0.0f
                };

                glLoadIdentity();
                glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, line_coords);
                glDrawArrays(GL_LINE_LOOP, 0, 4);
            }

            glDisableClientState(GL_VERTEX_ARRAY);
            eglSwapBuffers(g_disp, g_surf);

        }
        else {
            fprintf(stderr, "failed to decompress frame\n");
        }
    }

    // shutdown opengl subsystem
    printf("shutting down gl\n");
    eglDestroyContext(g_disp, g_ctx);
    eglDestroySurface(g_disp, g_surf);
    eglMakeCurrent(g_disp, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglTerminate(g_disp);

    // shutdown video subsystem
    printf("shutting down video\n");
    video_shutdown();
    free(rgb_buff);
    free(jpg_buff);

    return EXIT_SUCCESS;
}

