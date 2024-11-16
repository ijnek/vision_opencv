/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  Copyright (c) 2018 Intel Corporation.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "module.hpp"
#include <string>

PyObject * mod_opencv;

bp::object
cvtColor2Wrap(bp::object obj_in, const std::string & encoding_in, const std::string & encoding_out)
{
  // Convert the Python input to an image
  cv::Mat mat_in;
  convert_to_CvMat2(obj_in.ptr(), mat_in);

  // Call cv_bridge for color conversion
  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
      std_msgs::msg::Header(), encoding_in, mat_in));

  cv::Mat mat = cv_bridge::cvtColor(cv_image, encoding_out)->image;

  return bp::object(boost::python::handle<>(pyopencv_from(mat)));
}

bp::object
cvtColorForDisplayWrap(
  bp::object obj_in,
  const std::string & encoding_in,
  const std::string & encoding_out,
  bool do_dynamic_scaling = false,
  double min_image_value = 0.0,
  double max_image_value = 0.0,
  int colormap = -1)
{
  // Convert the Python input to an image
  cv::Mat mat_in;
  convert_to_CvMat2(obj_in.ptr(), mat_in);

  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(
      std_msgs::msg::Header(), encoding_in, mat_in));

  cv_bridge::CvtColorForDisplayOptions options;
  options.do_dynamic_scaling = do_dynamic_scaling;
  options.min_image_value = min_image_value;
  options.max_image_value = max_image_value;
  options.colormap = colormap;
  cv::Mat mat = cv_bridge::cvtColorForDisplay(/*source=*/ cv_image,
      /*encoding_out=*/ encoding_out,
      /*options=*/ options)->image;

  return bp::object(boost::python::handle<>(pyopencv_from(mat)));
}

BOOST_PYTHON_FUNCTION_OVERLOADS(cvtColorForDisplayWrap_overloads, cvtColorForDisplayWrap, 3, 7)

int CV_MAT_CNWrap(int i)
{
  return CV_MAT_CN(i);
}

int CV_MAT_DEPTHWrap(int i)
{
  return CV_MAT_DEPTH(i);
}

BOOST_PYTHON_MODULE(cv_bridge_boost)
{
  do_numpy_import();
  mod_opencv = PyImport_ImportModule("cv2");

  // Wrap the function to get encodings as OpenCV types
  boost::python::def("getCvType", cv_bridge::getCvType);
  boost::python::def("cvtColor2", cvtColor2Wrap);
  boost::python::def("CV_MAT_CNWrap", CV_MAT_CNWrap);
  boost::python::def("CV_MAT_DEPTHWrap", CV_MAT_DEPTHWrap);
  boost::python::def("cvtColorForDisplay", cvtColorForDisplayWrap,
    cvtColorForDisplayWrap_overloads(
      boost::python::args("source", "encoding_in", "encoding_out", "do_dynamic_scaling",
      "min_image_value", "max_image_value", "colormap"),
      "Convert image to display with specified encodings.\n\n"
      "Args:\n"
      "  - source (numpy.ndarray): input image\n"
      "  - encoding_in (str): input image encoding\n"
      "  - encoding_out (str): encoding to which the image conveted\n"
      "  - do_dynamic_scaling (bool): flag to do dynamic scaling with min/max value\n"
      "  - min_image_value (float): minimum pixel value for dynamic scaling\n"
      "  - max_image_value (float): maximum pixel value for dynamic scaling\n"
      "  - colormap (int): colormap to use when converting to color image\n"
  ));
}

/* Taken from opencv/modules/python/src2/cv2.cpp */

#include "module.hpp"

#include "opencv2/core/types_c.h"

#include "opencv2/opencv_modules.hpp"

#include "pycompat.hpp"

static PyObject * opencv_error = 0;

static int failmsg(const char * fmt, ...)
{
  char str[1000];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  PyErr_SetString(PyExc_TypeError, str);
  return 0;
}

struct ArgInfo
{
  const char * name;
  bool outputarg;
  // more fields may be added if necessary

  ArgInfo(const char * name_, bool outputarg_)
  : name(name_),
    outputarg(outputarg_) {}

  // to match with older pyopencv_to function signature
  operator const char *() const {return name;}
};

class PyAllowThreads
{
public:
  PyAllowThreads()
  : _state(PyEval_SaveThread()) {}
  ~PyAllowThreads()
  {
    PyEval_RestoreThread(_state);
  }

private:
  PyThreadState * _state;
};

class PyEnsureGIL
{
public:
  PyEnsureGIL()
  : _state(PyGILState_Ensure()) {}
  ~PyEnsureGIL()
  {
    PyGILState_Release(_state);
  }

private:
  PyGILState_STATE _state;
};

#define ERRWRAP2(expr) \
  try \
  { \
    PyAllowThreads allowThreads; \
    expr; \
  } \
  catch (const cv::Exception & e) \
  { \
    PyErr_SetString(opencv_error, e.what()); \
    return 0; \
  }

using namespace cv;


[[gnu::unused]] static PyObject * failmsgp(const char * fmt, ...)
{
  char str[1000];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  PyErr_SetString(PyExc_TypeError, str);
  return 0;
}

class NumpyAllocator : public MatAllocator
{
public:
  NumpyAllocator() {stdAllocator = Mat::getStdAllocator();}
  ~NumpyAllocator() {}

// To compile openCV3 with OpenCV4 APIs.
#ifndef OPENCV_VERSION_4
#define AccessFlag int
#endif

  UMatData * allocate(PyObject * o, int dims, const int * sizes, int type, size_t * step) const
  {
    UMatData * u = new UMatData(this);
    u->data = u->origdata =
      reinterpret_cast<uchar *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(o)));
    npy_intp * _strides = PyArray_STRIDES(reinterpret_cast<PyArrayObject *>(o));
    for (int i = 0; i < dims - 1; i++) {
      step[i] = (size_t)_strides[i];
    }
    step[dims - 1] = CV_ELEM_SIZE(type);
    u->size = sizes[0] * step[0];
    u->userdata = o;
    return u;
  }

  UMatData * allocate(
    int dims0, const int * sizes, int type, void * data, size_t * step, AccessFlag flags,
    UMatUsageFlags usageFlags) const
  {
    if (data != 0) {
      CV_Error(Error::StsAssert, "The data should normally be NULL!");
      // probably this is safe to do in such extreme case
      return stdAllocator->allocate(dims0, sizes, type, data, step, flags, usageFlags);
    }
    PyEnsureGIL gil;

    int depth = CV_MAT_DEPTH(type);
    int cn = CV_MAT_CN(type);
    const int f = static_cast<int>(sizeof(size_t) / 8);
    int typenum = depth == CV_8U ? NPY_UBYTE : depth == CV_8S ? NPY_BYTE :
      depth == CV_16U ? NPY_USHORT : depth == CV_16S ? NPY_SHORT :
      depth == CV_32S ? NPY_INT : depth == CV_32F ? NPY_FLOAT :
      depth == CV_64F ? NPY_DOUBLE : f * NPY_ULONGLONG + (f ^ 1) * NPY_UINT;
    int i, dims = dims0;
    cv::AutoBuffer<npy_intp> _sizes(dims + 1);
    for (i = 0; i < dims; i++) {
      _sizes[i] = sizes[i];
    }
    if (cn > 1) {
      _sizes[dims++] = cn;
    }
    PyObject * o = PyArray_SimpleNew(dims, _sizes, typenum);
    if (!o) {
      CV_Error_(Error::StsError,
        ("The numpy array of typenum=%d, ndims=%d can not be created", typenum, dims));
    }
    return allocate(o, dims0, sizes, type, step);
  }

  bool allocate(UMatData * u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const
  {
    return stdAllocator->allocate(u, accessFlags, usageFlags);
  }

  void deallocate(UMatData * u) const
  {
    if (u) {
      PyEnsureGIL gil;
      PyObject * o = reinterpret_cast<PyObject *>(u->userdata);
      Py_XDECREF(o);
      delete u;
    }
  }

  const MatAllocator * stdAllocator;
};

NumpyAllocator g_numpyAllocator;


template<typename T>
static
bool pyopencv_to(PyObject * obj, T & p, const char * name = "<unknown>");

template<typename T>
static
PyObject * pyopencv_from(const T & src);

enum { ARG_NONE = 0, ARG_MAT = 1, ARG_SCALAR = 2 };

// special case, when the convertor needs full ArgInfo structure
static bool pyopencv_to(PyObject * o, Mat & m, const ArgInfo info)
{
  // to avoid PyArray_Check() to crash even with valid array
  do_numpy_import();


  bool allowND = true;
  if (!o || o == Py_None) {
    if (!m.data) {
      m.allocator = &g_numpyAllocator;
    }
    return true;
  }

  if (PyInt_Check(o) ) {
    double v[] = {static_cast<double>(PyInt_AsLong(reinterpret_cast<PyObject *>(o))), 0., 0., 0.};
    m = Mat(4, 1, CV_64F, v).clone();
    return true;
  }
  if (PyFloat_Check(o) ) {
    double v[] = {PyFloat_AsDouble(reinterpret_cast<PyObject *>(o)), 0., 0., 0.};
    m = Mat(4, 1, CV_64F, v).clone();
    return true;
  }
  if (PyTuple_Check(o) ) {
    int i, sz = static_cast<int>(PyTuple_Size(reinterpret_cast<PyObject *>(o)));
    m = Mat(sz, 1, CV_64F);
    for (i = 0; i < sz; i++) {
      PyObject * oi = PyTuple_GET_ITEM(o, i);
      if (PyInt_Check(oi) ) {
        m.at<double>(i) = static_cast<double>(PyInt_AsLong(oi));
      } else if (PyFloat_Check(oi) ) {
        m.at<double>(i) = static_cast<double>(PyFloat_AsDouble(oi));
      } else {
        failmsg("%s is not a numerical tuple", info.name);
        m.release();
        return false;
      }
    }
    return true;
  }

  if (!PyArray_Check(o) ) {
    failmsg("%s is not a numpy array, neither a scalar", info.name);
    return false;
  }

  PyArrayObject * oarr = reinterpret_cast<PyArrayObject *>(o);

  bool needcopy = false, needcast = false;
  int typenum = PyArray_TYPE(oarr), new_typenum = typenum;
  int type = typenum == NPY_UBYTE ? CV_8U :
    typenum == NPY_BYTE ? CV_8S :
    typenum == NPY_USHORT ? CV_16U :
    typenum == NPY_SHORT ? CV_16S :
    typenum == NPY_INT ? CV_32S :
    typenum == NPY_INT32 ? CV_32S :
    typenum == NPY_FLOAT ? CV_32F :
    typenum == NPY_DOUBLE ? CV_64F : -1;

  if (type < 0) {
    if (typenum == NPY_INT64 || typenum == NPY_UINT64 || type == NPY_LONG) {
      needcopy = needcast = true;
      new_typenum = NPY_INT;
      type = CV_32S;
    } else {
      failmsg("%s data type = %d is not supported", info.name, typenum);
      return false;
    }
  }

#ifndef CV_MAX_DIM
  const int CV_MAX_DIM = 32;
#endif

  int ndims = PyArray_NDIM(oarr);
  if (ndims >= CV_MAX_DIM) {
    failmsg("%s dimensionality (=%d) is too high", info.name, ndims);
    return false;
  }

  int size[CV_MAX_DIM + 1];
  size_t step[CV_MAX_DIM + 1];
  size_t elemsize = CV_ELEM_SIZE1(type);
  const npy_intp * _sizes = PyArray_DIMS(oarr);
  const npy_intp * _strides = PyArray_STRIDES(oarr);
  bool ismultichannel = ndims == 3 && _sizes[2] <= CV_CN_MAX;

  for (int i = ndims - 1; i >= 0 && !needcopy; i--) {
    // these checks handle cases of
    //  a) multi-dimensional (ndims > 2) arrays, as well as simpler 1- and 2-dimensional cases
    //  b) transposed arrays, where _strides[] elements go in non-descending order
    //  c) flipped arrays, where some of _strides[] elements are negative
    if ( (i == ndims - 1 && (size_t)_strides[i] != elemsize) ||
      (i < ndims - 1 && _strides[i] < _strides[i + 1]) )
    {
      needcopy = true;
    }
  }

  if (ismultichannel && _strides[1] != (npy_intp)elemsize * _sizes[2]) {
    needcopy = true;
  }

  if (needcopy) {
    if (info.outputarg) {
      failmsg(
        "Layout of the output array %s is incompatible with \
         cv::Mat (step[ndims-1] != elemsize or step[1] != elemsize*nchannels)",
        info.name);
      return false;
    }

    if (needcast) {
      o = PyArray_Cast(oarr, new_typenum);
      oarr = reinterpret_cast<PyArrayObject *>(o);
    } else {
      oarr = PyArray_GETCONTIGUOUS(oarr);
      o = reinterpret_cast<PyObject *>(oarr);
    }

    _strides = PyArray_STRIDES(oarr);
  }

  for (int i = 0; i < ndims; i++) {
    size[i] = static_cast<int>(_sizes[i]);
    step[i] = (size_t)_strides[i];
  }

  // handle degenerate case
  if (ndims == 0) {
    size[ndims] = 1;
    step[ndims] = elemsize;
    ndims++;
  }

  if (ismultichannel) {
    ndims--;
    type |= CV_MAKETYPE(0, size[2]);
  }

  if (ndims > 2 && !allowND) {
    failmsg("%s has more than 2 dimensions", info.name);
    return false;
  }

  m = Mat(ndims, size, type, PyArray_DATA(oarr), step);
  m.u = g_numpyAllocator.allocate(o, ndims, size, type, step);
  m.addref();

  if (!needcopy) {
    Py_INCREF(o);
  }
  m.allocator = &g_numpyAllocator;

  return true;
}

template<>
bool pyopencv_to(PyObject * o, Mat & m, const char * name)
{
  return pyopencv_to(o, m, ArgInfo(name, 0));
}

PyObject * pyopencv_from(const Mat & m)
{
  if (!m.data) {
    Py_RETURN_NONE;
  }
  Mat temp, * p = const_cast<Mat *>(&m);
  if (!p->u || p->allocator != &g_numpyAllocator) {
    temp.allocator = &g_numpyAllocator;
    ERRWRAP2(m.copyTo(temp));
    p = &temp;
  }
  PyObject * o = reinterpret_cast<PyObject *>(p->u->userdata);
  Py_INCREF(o);
  return o;
}

int convert_to_CvMat2(const PyObject * o, cv::Mat & m)
{
  pyopencv_to(const_cast<PyObject *>(o), m, "unknown");
  return 0;
}
