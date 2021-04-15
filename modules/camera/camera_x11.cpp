/*************************************************************************/
/*  camera_x11.cpp                                                        */
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

#include "camera_x11.h"
#include "servers/camera/camera_feed.h"

#include <dirent.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cstddef>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>

#include <libv4l2.h>
#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

// formats that can be used
// mainly necessary if libv4l2 is not available
std::vector<int> supported_formats{
    V4L2_PIX_FMT_RGB24
};

int V4l2_Device::xioctl(unsigned long int request, void *arg){
    int r;
	do {
        r = this->funcs->ioctl(this->fd, request, arg);
	} while (-1 == r && (EINTR == errno || errno == EAGAIN));
	return r;
};

V4l2_Device::V4l2_Device(const char *dev_name, struct v4l2_funcs *funcs) {
    this->dev_name = std::string(dev_name);
    this->funcs = funcs;
    this->use_libv4l2 = funcs->libv4l2;
};

bool V4l2_Device::open() {
    struct stat st;
    if (-1 == stat(dev_name.c_str(), &st) || (!S_ISCHR(st.st_mode))){
    #ifdef DEBUG_ENABLED
	    print_line("Device name " + String(dev_name.c_str()) + " not available.");
    #endif
        return false;
    }
    if (fd != -1){
        funcs->close(fd);
    }

    // open device
	fd = funcs->open(dev_name.c_str(), O_RDWR | O_NONBLOCK, 0);

	if (-1 == fd) {
    #ifdef DEBUG_ENABLED
	    print_line("Cannot open device " + String(dev_name.c_str()) + ".");
    #endif
        return false;
    }

    CLEAR(cap);
    if (-1 == xioctl(VIDIOC_QUERYCAP, &cap)) {
    #ifdef DEBUG_ENABLED
	    print_line("Cannot query capabilities for " + String(dev_name.c_str()) + ".");
    #endif
        funcs->close(fd);
        return false;
    }

    // Check if it is a videocapture device
    if ((cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) != V4L2_CAP_VIDEO_CAPTURE){
    #ifdef DEBUG_ENABLED
	    print_line(String(dev_name.c_str()) + " is no video capture device.");
    #endif
        funcs->close(fd);
        return false;
    }

    // if its just a meta data device (unused)
    if (((cap.device_caps & V4L2_CAP_META_OUTPUT) == V4L2_CAP_META_OUTPUT)
        || ((cap.device_caps & V4L2_CAP_META_CAPTURE) == V4L2_CAP_META_CAPTURE)) {
    #ifdef DEBUG_ENABLED
	    print_line(String(dev_name.c_str()) + " is just a meta information device.");
    #endif        
        funcs->close(fd);
        return false;
    }

    // How the image data can be accessed
    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
        type = TYPE_IO_MMAP;
    } else if((cap.capabilities & V4L2_CAP_READWRITE) == V4L2_CAP_READWRITE) {
        type = TYPE_IO_READ;
    } else {
    #ifdef DEBUG_ENABLED
	    print_line(String(dev_name.c_str()) + " has no capability to capture frames.");
    #endif
        funcs->close(fd);
        return false;
    }

    // Check if device supports supported formats
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Get default size and format
    if (-1 == this->xioctl(VIDIOC_G_FMT, &fmt)){
    #ifdef DEBUG_ENABLED
	    print_line("Cannot grab default format from " + String(dev_name.c_str()) + ".");
    #endif
        funcs->close(fd);
        return false;
    }
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    bool found = false;
    for(unsigned int i = 0; i < supported_formats.size(); ++i){
        fmt.fmt.pix.pixelformat = supported_formats[i];
        if(-1 == xioctl(VIDIOC_S_FMT, &fmt)) {
            if (errno == EBUSY) {
            #ifdef DEBUG_ENABLED
                print_line(String(dev_name.c_str()) + " is busy. Check for format.");
            #endif
                // If device is busy, check if it can use the format
                // Device is still available even if it is busy
                if (-1 == xioctl(VIDIOC_TRY_FMT, &fmt)) {
                    continue;
                }
            } else {
                continue;
            }
        }
        found = true;
        break;
    }
    if (!found) {
    #ifdef DEBUG_ENABLED
	    print_line(String(dev_name.c_str()) + " has no supported pixelformat.");
    #endif
        funcs->close(fd);
        return false;
    }

    /* Buggy driver paranoia. */
    unsigned int min = fmt.fmt.pix.width * 2;
    unsigned int bpl = fmt.fmt.pix.bytesperline;
    if (bpl < min)
        bpl = min;
    min = bpl * fmt.fmt.pix.height;
    buffer_size = fmt.fmt.pix.sizeimage;
    if (buffer_size < min)
        buffer_size = min;

    funcs->close(fd);
    fd = funcs->open(dev_name.c_str(), O_RDWR | O_NONBLOCK, 0);
    if(fd == -1) {
        return false;
    }

    name = String((const char*) cap.card) + String(" (") + String(dev_name.c_str()) + String(")");
    opened = true;

    // Now the device is opened... To start streaming the fmt must be set and the buffers must be prepared.
    return true;
}

bool V4l2_Device::close() {
    if(buffer_available){
        cleanup_buffers();
    }
    if (fd != -1) {
        funcs->close(fd);
        fd = -1;
        opened = false;
    }
    return true;
}

bool V4l2_Device::request_buffers() {
    if(-1 == xioctl(VIDIOC_S_FMT, &fmt)) {
    #ifdef DEBUG_ENABLED
	    print_line("Cannot set format for " + String(dev_name.c_str()) + ".");
    #endif
        return false;
    }

    width = 0;
    height = 0;

    struct v4l2_requestbuffers req;

    switch(type){
    case TYPE_IO_READ:
        {
            buffers = (V4l2_Device::buffer*) calloc(1, sizeof(*buffers));

            if (!buffers) {
                #ifdef DEBUG_ENABLED
	                print_line(String(dev_name.c_str()) + ": Out of memory");
                #endif
                return false;
            }

            buffers[0].length = buffer_size;
            buffers[0].start = malloc(buffer_size);

            if (!buffers[0].start) {
                #ifdef DEBUG_ENABLED
	                print_line(String(dev_name.c_str()) + ": Out of memory");
                #endif
                return false;
            }
            break;
        }
    case TYPE_IO_MMAP:
        {
            CLEAR(req);

            req.count = 4;
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.memory = V4L2_MEMORY_MMAP;

            int r = xioctl(VIDIOC_REQBUFS, &req);
            if (r == -1 && EINVAL == errno) {
                type = TYPE_IO_USRPTR;
                break;
                // the switch statement should go to the next level
            } else if (r == -1) {
                return false;
            } else {
                if (req.count < 2)
                    // fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                    return false;
                buffers = (V4l2_Device::buffer*) calloc(req.count, sizeof(*buffers));

                if (!buffers) {
                    // fprintf(stderr, "Out of memory\n");
                    return false;
                }

                for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                    CLEAR(buf);

                    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory      = V4L2_MEMORY_MMAP;
                    buf.index       = n_buffers;

                    if (-1 == xioctl(VIDIOC_QUERYBUF, &buf))
                        return false;

                    buffers[n_buffers].length = buf.length;
                    buffers[n_buffers].start = funcs->mmap(
                        NULL /* start anywhere */,
                        buf.length,
                        PROT_READ | PROT_WRITE /* required */,
                        MAP_SHARED /* recommended */,
                        fd, buf.m.offset
                    );

                    if (MAP_FAILED == buffers[n_buffers].start)
                        return false;
                }
                break;
            }
        }
    default:
        break;
    }
    switch(type) {
    case TYPE_IO_NONE:
        return false;
    case TYPE_IO_USRPTR:
        {
            CLEAR(req);

            req.count  = 4;
            req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(VIDIOC_REQBUFS, &req)) {
                return false;
            }

            buffers = (V4l2_Device::buffer*) calloc(4, sizeof(*buffers));

            if (!buffers) {
                //fprintf(stderr, "Out of memory\n");
                return false;
            }

            for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                    // fprintf(stderr, "Out of memory\n");
                    return false;
                }
            }
            break;
        }
    default:
        break;
    }
    buffer_available = true;
    return true;
}

void V4l2_Device::cleanup_buffers() {
    switch (type) {
    case TYPE_IO_READ:
        {
            free(buffers[0].start);
            break;
        }
    case TYPE_IO_MMAP:
        {
            for (unsigned int i = 0; i < n_buffers; ++i)
                if (-1 == funcs->munmap(buffers[i].start, buffers[i].length))
                    ;//errno_exit("munmap");
                break;
        }
    case TYPE_IO_USRPTR:
        {
            for (unsigned int i = 0; i < n_buffers; ++i)
                free(buffers[i].start);
            break;
        }
    default:
        break;
    }
    free(buffers);
    buffer_available = false;
}


V4l2_Device::~V4l2_Device() {
    if(this->opened){
        this->close();
    }
}

bool V4l2_Device::start_streaming(Ref<CameraFeed> feed) {
    enum v4l2_buf_type b_type;

    switch (type) {
    case TYPE_IO_MMAP:
        {
            for (unsigned int i = 0; i < n_buffers; ++i) {
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(VIDIOC_QBUF, &buf))
                    return false;
            }
            b_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(VIDIOC_STREAMON, &b_type))
                return false;
            break;
        }
    case TYPE_IO_USRPTR:
        {
            for (unsigned int i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long) buffers[i].start;
                buf.length = buffers[i].length;

                if (-1 == xioctl(VIDIOC_QBUF, &buf))
                    return false;
            }
            b_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == xioctl(VIDIOC_STREAMON, &b_type))
                return false;
            break;
        }
    default:
        break;
    }

    streaming = true;
    stream_thread = std::thread(&V4l2_Device::stream_image, this, feed);
    return true;
};

void V4l2_Device::stop_streaming() {
    streaming = false;
    if (stream_thread.joinable())
        stream_thread.join();

    switch (type) {
    case TYPE_IO_MMAP:
    case TYPE_IO_USRPTR:
        enum v4l2_buf_type b_type;
        b_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(VIDIOC_STREAMOFF, &b_type))
            ;
        break;
    default:
        break;
    }
}

void V4l2_Device::stream_image(Ref<CameraFeed> feed) {
    fd_set fds;
    struct timeval tv;
    int r;
    while(this->streaming){
        do {
            FD_ZERO(&fds);
            FD_SET(this->fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);
        } while(-1 == r && (EINTR == errno || EAGAIN == errno));
        if (r <= 0) {
            return;
            // errno_exit("select");
        }

        switch (type) {
        case TYPE_IO_READ:
            {
                if (-1 == funcs->read(fd, buffers[0].start, buffers[0].length)) {
                    switch (errno) {
                    case EAGAIN:
                        continue;
                    case EIO: // could be ignored somehow?
                    default:
                        return;
                    }
                }

                get_image(feed, (uint8_t*) (buffers[0].start));
                break;
            }
        case TYPE_IO_MMAP:
            {
                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl(VIDIOC_DQBUF, &buf)) {
                    switch (errno) {
                    case EAGAIN:
                        continue;
                    case EIO: // could be ignored
                    default:
                        return;
                    }
                }

                get_image(feed, (uint8_t *) (buffers[buf.index].start));

                if (-1 == xioctl(VIDIOC_QBUF, &buf))
                    return;
                break;
            }
        case TYPE_IO_USRPTR:
            {
                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;

                if (-1 == xioctl(VIDIOC_DQBUF, &buf)) {
                    switch (errno) {
                    case EAGAIN:
                        continue;
                    case EIO: // could be ignored
                    default:
                        return;
                    }
                }

                for (unsigned int i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

                get_image(feed, (uint8_t *) (buf.m.userptr));

                if (-1 == xioctl(VIDIOC_QBUF, &buf))
                    return;
                break;
            }
        default:
            break;
        }
    }
}

// This one is entirely format dependent
void V4l2_Device::get_image(Ref<CameraFeed> feed, uint8_t* buffer) {
    Ref<Image> img;
    img.instance();

    if (-1 == xioctl(VIDIOC_G_FMT, &fmt)){
        return;
    }

    if( width != fmt.fmt.pix.width || height != fmt.fmt.pix.height ) {
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        img_data.resize(width * height * 3);
    }

    PoolVector<uint8_t>::Write w = img_data.write();
    // TODO: Buffer is 1024 Byte longer?
    memcpy(w.ptr(), buffer, width*height*3);

    img->create(width, height, 0, Image::FORMAT_RGB8, img_data);
    feed->set_RGB_img(img);
}

//////////////////////////////////////////////////////////////////////////
// CameraFeedX11 - Subclass for camera feeds in Linux

V4l2_Device *CameraFeedX11::get_device() const {
	return device;
};

CameraFeedX11::CameraFeedX11() {
	device = NULL;
};

void CameraFeedX11::set_device(V4l2_Device *p_device) {
	device = p_device;

	// get some info
	name = device->name;
	position = CameraFeed::FEED_UNSPECIFIED;
	// if ([p_device position] == AVCaptureDevicePositionBack) {
	// 	position = CameraFeed::FEED_BACK;
	// } else if ([p_device position] == AVCaptureDevicePositionFront) {
	// 	position = CameraFeed::FEED_FRONT;
	// };
};

CameraFeedX11::~CameraFeedX11() {
    this->deactivate_feed();
    if (device != NULL) {
        delete device;
        device = NULL;
    }
};

bool CameraFeedX11::activate_feed() {
	if (!device->streaming) {
        if(!device->open()) {
            return false;
        }
        if(!device->request_buffers()) {
            device->close();
            return false;
        }
        if(!device->start_streaming(this)) {
            device->cleanup_buffers();
            device->close();
            return false;
        }
	};

	return true;
};

void CameraFeedX11::deactivate_feed() {
	// end camera capture if we have one
	if (device->streaming) {
        device->stop_streaming();
        device->cleanup_buffers();
        device->close();
	};
};

void CameraX11::update_feeds() {
    DIR *dir;
    struct dirent *ent;
    auto devs = std::vector<std::string>();
    if ((dir = opendir ("/dev/")) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            if(strncmp (ent->d_name, "video", 5) == 0) {
                devs.push_back(std::string("/dev/")+std::string(ent->d_name));
            }
        }
        closedir (dir);
    }
    std::sort(devs.begin(), devs.end());
    for(unsigned int i = 0; i < devs.size(); ++i) {
        V4l2_Device *dev = new V4l2_Device(devs[i].c_str(), &this->funcs);
        if (dev->open()) {
            dev->close();
            Ref<CameraFeedX11> newfeed;
			newfeed.instance();
			newfeed->set_device(dev);

			// assume display camera so inverse
			Transform2D transform = Transform2D(-1.0, 0.0, 0.0, -1.0, 1.0, 1.0);
			newfeed->set_transform(transform);

			add_feed(newfeed);
        }
    }
};

CameraX11::CameraX11() {
	// // Find available cameras we have at this time
    this->devices = std::vector<V4l2_Device*>();
    this->libv4l2 = dlopen("libv4l2.so", RTLD_NOW);

    if (libv4l2 == NULL) {
        this->funcs.open = &open;
        this->funcs.close = &close;
        this->funcs.dup = &dup;
        this->funcs.ioctl = &ioctl;
        this->funcs.read = &read;
        this->funcs.mmap = &mmap;
        this->funcs.munmap = &munmap;
        this->funcs.libv4l2 = false;
    } else {
        this->funcs.open = (int (*)(const char*, int, ...)) dlsym(libv4l2, "v4l2_open");
        this->funcs.close = (int (*)(int)) dlsym(libv4l2, "v4l2_close");
        this->funcs.dup = (int (*)(int)) dlsym(libv4l2, "v4l2_dup");
        this->funcs.ioctl = (int (*)(int, unsigned long int, ...)) dlsym(libv4l2, "v4l2_ioctl");
        this->funcs.read = (long int (*)(int, void*, size_t)) dlsym(libv4l2, "v4l2_read");
        this->funcs.mmap = (void* (*) (void*, size_t, int, int, int, int64_t)) dlsym(libv4l2, "v4l2_mmap");
        this->funcs.munmap = (int (*) (void*, size_t)) dlsym(libv4l2, "v4l2_munmap");
        this->funcs.libv4l2 = true;
    }

	update_feeds();

	// // should only have one of these....
	// device_notifications = [[MyDeviceNotifications alloc] initForServer:this];
};

CameraX11::~CameraX11() {
    if (this->libv4l2 != NULL) {
        dlclose(this->libv4l2);
    }
	// [device_notifications release];
};
