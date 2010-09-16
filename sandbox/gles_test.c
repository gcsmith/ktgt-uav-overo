#include <stdio.h>
#include <stdlib.h>
#include <GLES/egl.h>
#include <GLES/gl.h>

int main(int argc, char *argv[])
{
    EGLDisplay display;
    EGLConfig cfg = 0;
    EGLContext context = 0;
    EGLSurface surface = 0;
    EGLint major, minor;
    EGLint num_cfg;

    // get the default display handle
    display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (EGL_NO_DISPLAY == display)
    {
        printf("error: unable to get display\n");
        return 1;
    }

    // initialize the EGL library and print version
    if (EGL_FALSE == eglInitialize(display, &major, &minor))
    {
        printf("error: failed to initialize EGL\n");
        return 1;
    }
    printf("initialized egl v%d.%d\n", major, minor);

    // bind EGL to the OpenGL ES API
    if (EGL_FALSE == eglBindAPI(EGL_OPENGL_ES_API))
    {
        printf("error: failed to bind OpenGL ES API\n");
        return 1;
    }

    // specify a 16-bit RGB-565 color config
    const EGLint KColorRGB565AttribList[] =
    {
        EGL_RED_SIZE,        5,
        EGL_GREEN_SIZE,      6,
        EGL_BLUE_SIZE,       5,
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES_BIT,
        EGL_NONE
    };

    eglChooseConfig(display, KColorRGB565AttribList, &cfg, 1, &num_cfg);
    if (0 == num_cfg)
    {
        printf("error: no matching configs found\n");
        return 1;
    }
    printf("found %d matching config(s)\n", num_cfg);

    // create the window surface
    surface = eglCreateWindowSurface(display, cfg, (NativeWindowType)0, NULL);
    if (EGL_NO_SURFACE == surface)
    {
        printf("error: failed to create window surface\n");
        return 1;
    }

    // create and bind the rendering context
    context = eglCreateContext(display, cfg, NULL, NULL);
    if (EGL_NO_CONTEXT == context)
    {
        printf("error: failed to create rendering context\n");
        return 1;
    }

    if (EGL_FALSE == eglMakeCurrent(display, surface, surface, context))
    {
        printf("error: failed to bind rendering context\n");
        return 1;
    }

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // perform shutdown
    printf("shutting down gl\n");
    eglDestroyContext(display, context);
    eglDestroySurface(display, surface);
    eglTerminate(display);
    return 0;
}

