#ifndef SCENEGRAPH_CONFIG_H
#define SCENEGRAPH_CONFIG_H

/// Platform
#define _UNIX_
/* #undef _WIN_ */
/* #undef _OSX_ */
#define _LINUX_
/* #undef _ANDROID_ */

/// Configured libraries
/* #undef HAVE_GLES */
/* #undef HAVE_GLUES */
#define HAVE_PANGOLIN
#define HAVE_ASSIMP
/* #undef HAVE_DEVIL */
/* #undef HAVE_PNG */
#define HAVE_JPEG
#define HAVE_TIFF

#endif //SCENEGRAPH_CONFIG_H
