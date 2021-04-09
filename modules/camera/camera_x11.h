/*************************************************************************/
/*  camera_lin.h                                                         */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2021 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2021 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef CAMERAX11_H
#define CAMERAX11_H
#include <vector>
#include <stdint.h>

#include "servers/camera_server.h"
class V4l2_Device;

struct v4l2_funcs {
    int (*open)(const char *file, int oflag, ...);
    int (*close)(int fd);
    int (*dup)(int fd);
    int (*ioctl)(int fd, unsigned long int request, ...);
    long int (*read)(int fd, void* buffer, size_t n);
    void* (*mmap)(void *start, size_t length, int prot, int flags, int fd, int64_t offset);
    int (*munmap)(void *_start, size_t length);
	bool libv4l2;
};

class CameraX11 : public CameraServer {
private:
	struct v4l2_funcs funcs;
	void* libv4l2;
public:
	CameraX11();
	~CameraX11();
	std::vector<V4l2_Device*> devices;

	void update_feeds();
};

#endif /* CAMERAX11_H */
