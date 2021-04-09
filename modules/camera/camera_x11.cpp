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
#include <libv4l2.h>
#include <linux/videodev2.h>
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

#define IO_MMAP 1
#define IO_READ 2

static void errno_exit(const char* code){
    printf(code);
    exit(1);
}

class V4l2_Device {
    private:
        struct buffer {
            void   *start;
            size_t length;
        } *buffers;
        unsigned int n_buffers;
        struct v4l2_buffer buf;
        int fd = -1;
        void* libv4l2_handle;
        int xioctl(unsigned long int request, void *arg);
        struct v4l2_funcs *funcs;

        std::thread stream_thread;
        void stream_image(Ref<CameraFeed> feed);
        PoolVector<uint8_t> img_data;
        bool is_init = false;

    public:
        bool use_libv4l2;
        bool streaming = false;
        const char *dev_name;
        struct v4l2_capability cap;

        int width, height;
        static V4l2_Device* create_device(const char *dev_name, struct v4l2_funcs *funcs);

        V4l2_Device(const char *dev_name, int type, struct v4l2_funcs *funcs);
        ~V4l2_Device();

        void init();
        void uninit();

        void start_streaming(Ref<CameraFeed> feed);
        void stop_streaming();

};

V4l2_Device* V4l2_Device::create_device(const char *dev_name, struct v4l2_funcs *funcs) {
    V4l2_Device *device = NULL;
    struct stat st;
    int fd = -1;
    int type = 0;

    if (-1 == stat(dev_name, &st) || (!S_ISCHR(st.st_mode)) )
        return device;

	fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);

	if (-1 == fd)
        return device;

    struct v4l2_capability cap;
    CLEAR(cap);

    int r;
    do {
        r = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    } while (-1 == r && (EINTR == errno || errno == EAGAIN));
    if (r == -1) {
        close(fd);
        return device;
    }
    if (((cap.device_caps & V4L2_CAP_META_OUTPUT) == V4L2_CAP_META_OUTPUT)
        || ((cap.device_caps & V4L2_CAP_META_CAPTURE) == V4L2_CAP_META_CAPTURE)) {
        return device;
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
        type = IO_MMAP;
    } else if((cap.capabilities & V4L2_CAP_READWRITE) == V4L2_CAP_READWRITE) {
        type = IO_READ;
    } else {
        return device;
    }
    close(fd);
    return new V4l2_Device(dev_name, type, funcs);
}

int V4l2_Device::xioctl(unsigned long int request, void *arg){
    int r;
	do {
        r = this->funcs->ioctl(this->fd, request, arg);
	} while (-1 == r && (EINTR == errno || errno == EAGAIN));
	return r;
};

V4l2_Device::V4l2_Device(const char *dev_name, int type, struct v4l2_funcs *funcs) {
    this->dev_name = dev_name;
    this->funcs = funcs;
    this->fd = this->funcs->open(dev_name, O_RDWR | O_NONBLOCK, 0);
    this->use_libv4l2 = funcs->libv4l2;
    CLEAR(this->cap);
    this->xioctl(VIDIOC_QUERYCAP, &this->cap);
    //this->init();
};

V4l2_Device::~V4l2_Device() {
    this->uninit();
    if (-1 == this->funcs->close(this->fd))
		errno_exit("close");
	this->fd = -1;
}

void V4l2_Device::init() {
   	struct v4l2_format fmt;

	CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == this->xioctl(VIDIOC_G_FMT, &fmt))
		errno_exit("VIDIOC_G_FMT");

    //printf("%s\n", fmt.fmt.pix.pixelformat);
    //printf("%s\n", V4L2_PIX_FMT_BGR24);
    
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    if (-1 == this->xioctl(VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");

    this->width = fmt.fmt.pix.width;
    this->height = fmt.fmt.pix.height;
    
    struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == this->xioctl(VIDIOC_REQBUFS, &req)) {
		// if (EINVAL == errno) {
		// 	fprintf(stderr, "%s does not support "
		// 		 "memory mapping\n", dev_name);
		// 	exit(EXIT_FAILURE);
		// } else {
        errno_exit("VIDIOC_REQBUFS");
		// }
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n",
			 dev_name);
		exit(EXIT_FAILURE);
	}

	this->buffers = (V4l2_Device::buffer*) calloc(req.count, sizeof(*this->buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (this->n_buffers = 0; this->n_buffers < req.count; ++this->n_buffers) {
		CLEAR(this->buf);

		this->buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		this->buf.memory      = V4L2_MEMORY_MMAP;
		this->buf.index       = this->n_buffers;

		if (-1 == this->xioctl(VIDIOC_QUERYBUF, &this->buf))
			errno_exit("VIDIOC_QUERYBUF");

		this->buffers[this->n_buffers].length = this->buf.length;
		this->buffers[this->n_buffers].start =
			this->funcs->mmap(NULL /* start anywhere */,
			      this->buf.length,
			      PROT_READ | PROT_WRITE /* required */,
			      MAP_SHARED /* recommended */,
			      this->fd, this->buf.m.offset);

		if (MAP_FAILED == this->buffers[this->n_buffers].start)
			errno_exit("mmap");
	}
    is_init = true;
};

void V4l2_Device::uninit() {
    if (is_init) {
        for (unsigned int i = 0; i < this->n_buffers; ++i)
            if (-1 == this->funcs->munmap(this->buffers[i].start, this->buffers[i].length))
                errno_exit("munmap");

        free(this->buffers);
    }
    is_init = false;
}

void V4l2_Device::start_streaming(Ref<CameraFeed> feed) {
    for (unsigned int i = 0; i < this->n_buffers; ++i) {

        CLEAR(this->buf);
        this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->buf.memory = V4L2_MEMORY_MMAP;
        this->buf.index = i;

        if (-1 == this->xioctl(VIDIOC_QBUF, &this->buf))
            errno_exit("VIDIOC_QBUF");
    }

    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == this->xioctl(VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

    streaming = true;
    stream_thread = std::thread(&V4l2_Device::stream_image, this, feed);
};

void V4l2_Device::stop_streaming() {
    streaming = false;
    if (stream_thread.joinable())
        stream_thread.join();
    
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == this->xioctl(VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
};

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

            r = select(this->fd + 1, &fds, NULL, NULL, &tv);
        } while(-1 == r && (EINTR == errno || EAGAIN == errno));
        if (r <= 0) {
            errno_exit("select");
        }

        CLEAR(this->buf);
        this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->buf.memory = V4L2_MEMORY_MMAP;
        this->xioctl(VIDIOC_DQBUF, &this->buf);
        this->xioctl(VIDIOC_QBUF, &this->buf);

        Ref<Image> img;
        img.instance();

        img_data.resize(this->width * this->height * 3);

        PoolVector<uint8_t>::Write w = this->img_data.write();
        // TODO: Buffer is 1024 Byte longer?
		memcpy(w.ptr(), (uint8_t*) this->buffers[this->buf.index].start, this->width*this->height*3);

        img->create(this->width, this->height, 0, Image::FORMAT_RGB8, this->img_data);
        feed->set_RGB_img(img);
    }
}

//////////////////////////////////////////////////////////////////////////
// CameraFeedX11 - Subclass for camera feeds in OSX

class CameraFeedX11 : public CameraFeed {
private:
	V4l2_Device *device;

public:
	V4l2_Device *get_device() const;

	CameraFeedX11();
	~CameraFeedX11();

	void set_device(V4l2_Device *p_device);

	bool activate_feed();
	void deactivate_feed();
};

V4l2_Device *CameraFeedX11::get_device() const {
	return device;
};

CameraFeedX11::CameraFeedX11() {
	device = NULL;
};

void CameraFeedX11::set_device(V4l2_Device *p_device) {
	device = p_device;

	// get some info
	name = String((const char*) device->cap.card) + String(" (") + String(device->dev_name) + String(")");
	position = CameraFeed::FEED_UNSPECIFIED;
	// if ([p_device position] == AVCaptureDevicePositionBack) {
	// 	position = CameraFeed::FEED_BACK;
	// } else if ([p_device position] == AVCaptureDevicePositionFront) {
	// 	position = CameraFeed::FEED_FRONT;
	// };
};

CameraFeedX11::~CameraFeedX11() {
    this->deactivate_feed();
};

bool CameraFeedX11::activate_feed() {
    device->init();
	if (!device->streaming) {
        device->start_streaming(this);
	};

	return true;
};

void CameraFeedX11::deactivate_feed() {
	// end camera capture if we have one
	if (device->streaming) {
        device->stop_streaming();
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
        V4l2_Device *dev = V4l2_Device::create_device(devs[i].c_str(), &this->funcs);
        if (dev != NULL) {
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
